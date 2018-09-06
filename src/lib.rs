#[macro_use(xplane_plugin)]
extern crate xplm;
use xplm::plugin::{Plugin, PluginInfo};

use xplm::data::borrowed::{DataRef, FindError};
use xplm::data::{ReadOnly, ReadWrite, DataRead, DataReadWrite, ArrayRead, ArrayReadWrite};
use xplm::flight_loop::{FlightLoop, LoopState};
use triple_buffer::{TripleBuffer, Input, Output};
use std::{thread, time};

mod buffered_control_data;
mod buffered_flight_data;
mod control_data;
mod flight_data;
mod quaternion;

use self::buffered_control_data::BufferedControlData;
use self::buffered_flight_data::BufferedFlightData;
use self::control_data::ControlData;
use self::flight_data::FlightData;
use self::quaternion::Quaternion;

extern crate triple_buffer;


struct FFSim {
    // overrides all flight control, i.e. throttle, control surfaces etc.
    //override_flightcontrol: DataRef<bool, ReadWrite>,
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
    barometer_inhg: DataRef<f32, ReadOnly>, // feet or metres??

    temperature_ambient_c: DataRef<f32, ReadOnly>, // temp outisde the aircraft
    temperature_le_c: DataRef<f32, ReadOnly>,      // temp at the leading edge of the wing
    air_density: DataRef<f32, ReadOnly>, // kg / m^3

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
            barometer_inhg: self.barometer_inhg.get(),
            ambient_temp: self.temperature_ambient_c.get(),
            air_density: self.air_density.get(),
        };

        self.plane_orientation_quaternion.get(&mut ret.plane_orientation_quaternion);

        ret
    }
}

impl Plugin for FFSim {
    type StartErr = FindError;
    fn start() -> Result<Self, Self::StartErr> {
        let (mut incoming_send, incoming_recv)
            = TripleBuffer::new(BufferedControlData::new()).split();
        let (outgoing_send, mut outgoing_recv)
            = TripleBuffer::new(BufferedFlightData::new()).split();

        let mut plugin = FFSim {
            //override_flightcontrol: DataRef::find("sim/operation/override/override_flightcontrol")?.writeable()?,
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
            barometer_inhg: DataRef::find("sim/weather/barometer_current_inhg")?,
            temperature_ambient_c: DataRef::find("sim/weather/temperature_ambient_c")?,
            temperature_le_c: DataRef::find("sim/weather/temperature_le_c")?,
            air_density: DataRef::find("sim/weather/rho")?,

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

        // Read control inputs and write flight data to the buffers every flight cycle
        let mut flight_loop = FlightLoop::new(|_loop_state: &mut LoopState| {
            // `PLUGIN` is a global created by `xplane_plugin!`
            // It *should* be safe to take an exclusive reference,
            // since X-Plane does not call us concurrently.
            let plugin : &mut FFSim = unsafe { &mut *PLUGIN };

            // Read from triple buffer and update controls
            let control = *plugin.incoming.read();
            plugin.rudder.set(control.rudder);
            plugin.left_aileron.set(control.left_aileron);
            plugin.right_aileron.set(control.right_aileron);
            plugin.elevator.set(control.elevator);

            // Throttle is a bit trickier b/c it's an array,
            // but we only have one engine so we only set the
            // first element.
            let mut throttle_buf = [0.0; 8];
            throttle_buf[0] = control.throttle;
            plugin.throttle.set(&mut throttle_buf);

            // Write flight data into triple buffer
            let flight_data = plugin.get_data();
            plugin.outgoing.write(flight_data);
        });
        flight_loop.schedule_immediate();

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
