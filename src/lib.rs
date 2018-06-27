#[macro_use(xplane_plugin)]
extern crate xplm;
use xplm::plugin::{Plugin, PluginInfo};

use xplm::data::borrowed::{DataRef, FindError};
use xplm::data::{ReadOnly, ReadWrite, DataRead, DataReadWrite, ArrayRead, StringRead};
use triple_buffer::{TripleBuffer, Input, Output};
use std::{thread, time};
use std::sync::Arc;

#[macro_use]
extern crate lazy_static;
extern crate triple_buffer;

// See FFSim struct for comments about these values
#[derive(Copy, Clone, Debug)]
struct BufferedFlightData {
    roll_rate: f32,
    pitch_rate: f32,
    yaw_rate: f32,

    true_theta: f32,
    mag_psi: f32,

    local_ax: f32,
    local_ay: f32,
    local_az: f32,
    plane_orientation_quaternion: [f32; 4],

    latitude: f64,
    longitude: f64,

    indicated_airspeed: f32,
    altitude: f32,
}

impl BufferedFlightData {
    fn new() -> Self {
        BufferedFlightData {
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
            true_theta: 0.0,
            mag_psi: 0.0,
            local_ax: 0.0,
            local_ay: 0.0,
            local_az: 0.0,
            plane_orientation_quaternion: [0.0; 4],
            latitude: 0.0,
            longitude: 0.0,
            indicated_airspeed: 0.0,
            altitude: 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct BufferedControlData {
    rudder: f32,
    left_aileron: f32,
    right_aileron: f32,
    elevator: f32,

    throttle: f32,
}

impl BufferedControlData {
    fn new() -> Self {
        BufferedControlData {
            rudder: 0.0,
            left_aileron: 0.0,
            right_aileron: 0.0,
            elevator: 0.0,
            throttle: 0.0,
        }
    }
}

struct FFSim {
    // overrides all flight control, i.e. throttle, control surfaces etc.
    override_flightcontrol: DataRef<bool, ReadWrite>,
    // overrides only control surfaces
    override_control_surfaces: DataRef<bool, ReadWrite>,
    // overrides only the throttle
    override_throttles: DataRef<bool, ReadWrite>,

    // control surfaces
    rudder: DataRef<f32, ReadWrite>, // XXX: Only the "left rudder" seems to have an effect on the plane
    left_aileron: DataRef<f32, ReadWrite>,
    right_aileron: DataRef<f32, ReadWrite>,
    elevator: DataRef<f32, ReadWrite>, // XXX: Again there seems to be one value mirrored across many datarefs

    throttle: DataRef<[f32], ReadWrite>,

    // flight controller inputs
    roll_rate: DataRef<f32, ReadOnly>,  // degrees/second
    pitch_rate: DataRef<f32, ReadOnly>, // ...
    yaw_rate: DataRef<f32, ReadOnly>,   // ...

    // XXX: Remember to negate these guys!!
    true_theta: DataRef<f32, ReadOnly>, // degrees, pitch
    mag_psi: DataRef<f32, ReadOnly>,    // degrees, yaw

    local_ax: DataRef<f32, ReadOnly>,
    local_ay: DataRef<f32, ReadOnly>,
    local_az: DataRef<f32, ReadOnly>,
    plane_orientation_quaternion: DataRef<[f32], ReadOnly>, // XXX: Remember to negate 4th component

    latitude: DataRef<f64, ReadOnly>,  // degrees
    longitude: DataRef<f64, ReadOnly>, // ...

    indicated_airspeed: DataRef<f32, ReadOnly>, // kias??
    altitude: DataRef<f32, ReadOnly>, // feet or metres??


    // Buffers for bidirectional communication
    incoming: Output<BufferedControlData>,
    outgoing: Input<BufferedFlightData>,
}

impl FFSim {
    fn get_data(&self) -> BufferedFlightData {
        // Throttle: we are only interested in first value
        let mut throttle_buf: [f32; 4] = [0.0; 4];
        self.throttle.get(&mut throttle_buf);

        let mut ret = BufferedFlightData {
            roll_rate: self.roll_rate.get(),
            pitch_rate: self.pitch_rate.get(),
            yaw_rate: self.yaw_rate.get(),
            true_theta: self.true_theta.get(),
            mag_psi: self.mag_psi.get(),
            local_ax: self.local_ax.get(),
            local_ay: self.local_ay.get(),
            local_az: self.local_az.get(),
            plane_orientation_quaternion: [0.0; 4],
            latitude: self.latitude.get(),
            longitude: self.longitude.get(),
            indicated_airspeed: self.indicated_airspeed.get(),
            altitude: self.altitude.get(),
        };

        self.plane_orientation_quaternion.get(&mut ret.plane_orientation_quaternion);

        ret
    }
}

impl Plugin for FFSim {
    type StartErr = FindError;
    fn start() -> Result<Self, Self::StartErr> {
        let (mut incoming_send, mut incoming_recv)
            = TripleBuffer::new(BufferedControlData::new()).split();
        let (mut outgoing_send, mut outgoing_recv)
            = TripleBuffer::new(BufferedFlightData::new()).split();

        let mut plugin = FFSim {
            override_flightcontrol: DataRef::find("sim/operation/override/override_flightcontrol")?.writeable()?,
            override_control_surfaces: DataRef::find("sim/operation/override/override_control_surfaces")?.writeable()?,
            override_throttles: DataRef::find("sim/operation/override/override_throttles")?.writeable()?,

            rudder: DataRef::find("sim/flightmodel/controls/ldruddef")?.writeable()?,
            left_aileron: DataRef::find("sim/flightmodel/controls/lail1def")?.writeable()?,
            right_aileron: DataRef::find("sim/flightmodel/controls/rail1def")?.writeable()?,
            elevator: DataRef::find("sim/flightmodel/controls/wing1l_elv1def")?.writeable()?,

            throttle: DataRef::find("sim/flightmodel/engine/ENGN_thro_use")?.writeable()?,

            // append "rad" to the end of the names to get these in radians
            roll_rate: DataRef::find("sim/flightmodel/position/P")?,
            pitch_rate: DataRef::find("sim/flightmodel/position/Q")?,
            yaw_rate: DataRef::find("sim/flightmodel/position/R")?,

            true_theta: DataRef::find("sim/flightmodel/position/true_theta")?,
            mag_psi: DataRef::find("sim/flightmodel/position/mag_psi")?,

            local_ax: DataRef::find("sim/flightmodel/position/local_ax")?,
            local_ay: DataRef::find("sim/flightmodel/position/local_ay")?,
            local_az: DataRef::find("sim/flightmodel/position/local_az")?,
            plane_orientation_quaternion: DataRef::find("sim/flightmodel/position/q")?,

            latitude: DataRef::find("sim/flightmodel/position/latitude")?,
            longitude: DataRef::find("sim/flightmodel/position/longitude")?,

            indicated_airspeed: DataRef::find("sim/flightmodel/position/indicated_airspeed")?, // XXX: Can have a "2" at the end?
            altitude: DataRef::find("sim/flightmodel/misc/h_ind")?, // XXX: Can have a "2" at the end?
                                                                           // Also this is barometric altitude; we _probably_ want this
                                                                           // instead of the absolute elevation above MSL.
            incoming: incoming_recv,
            outgoing: outgoing_send,
        };

        //plugin.override_flightcontrol.set(true);
        plugin.override_control_surfaces.set(true);
        plugin.override_throttles.set(true);

        // Thread to send flight data to controller
        thread::spawn(move|| {
            loop {
                println!("Read: {:?}", *outgoing_recv.read());
                thread::sleep(time::Duration::from_millis(100));
            }
        });

        // Thread to receive controller inputs
        thread::spawn(move|| {
            incoming_send.write(BufferedControlData::new());
            thread::sleep(time::Duration::from_millis(100));
        });

        println!("[FFSim] Plugin loaded");
        Ok(plugin)
    }

    fn info(&self) -> PluginInfo {
        PluginInfo {
            name: "FFSim".into(),
            signature: "au.edu.anu.ffsim".into(),
            description: "Flight simulator integration for FPGA/HIL".into(),
        }
    }

    fn enable(&mut self) {
        
    }
    
    fn disable(&mut self) {
        
    }
    
    fn stop(&mut self) {
        //self.override_flightcontrol.set(false);
        self.override_control_surfaces.set(false);
        self.override_throttles.set(false);

        //TODO: Stop threads
    }
}

xplane_plugin!(FFSim);
