#[macro_use(xplane_plugin)]
extern crate xplm;
use xplm::plugin::{Plugin, PluginInfo};

use xplm::data::borrowed::{DataRef, FindError};
use xplm::data::{ReadOnly, ReadWrite, DataRead, DataReadWrite, ArrayRead};
use xplm::flight_loop::FlightLoop;
use triple_buffer::{TripleBuffer, Input, Output};
use std::thread;
use std::sync::atomic::{AtomicBool, Ordering, ATOMIC_BOOL_INIT};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

mod buffered_control_data;
mod buffered_flight_data;
mod control_data;
mod flight_data;
mod quaternion;
mod comm;
mod flight_loop;

use self::buffered_control_data::BufferedControlData;
use self::buffered_flight_data::BufferedFlightData;
use self::control_data::ControlData;
use self::flight_data::FlightData;
use self::quaternion::Quaternion;
use self::flight_loop::flight_loop;

extern crate triple_buffer;
extern crate serial;

pub static STOP_THREADS: AtomicBool = ATOMIC_BOOL_INIT;

pub const NUM_LATENCY_MEASUREMENTS: usize = 100;
const SACRIFICE_LATENCY_MEASUREMENTS: usize = 100; // initially the latency is very erratic

pub struct FFSim {
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
    elevator1: DataRef<f32, ReadWrite>, // XXX: The elevators can be controlled independently ..
    elevator2: DataRef<f32, ReadWrite>,

    throttle: DataRef<[f32], ReadWrite>,

    // flight controller inputs
    roll_rate: DataRef<f32, ReadOnly>,  // degrees/second
    pitch_rate: DataRef<f32, ReadOnly>, // ...
    yaw_rate: DataRef<f32, ReadOnly>,   // ...

    true_theta: DataRef<f32, ReadOnly>, // degrees, pitch
    true_phi: DataRef<f32, ReadOnly>,   // degrees, roll
    mag_psi: DataRef<f32, ReadOnly>,    // degrees, yaw

    local_ax: DataRef<f32, ReadOnly>,
    local_ay: DataRef<f32, ReadOnly>,
    local_az: DataRef<f32, ReadOnly>,
    plane_orientation_quaternion: DataRef<[f32], ReadOnly>, // XXX: Remember to negate non-scalar parts

    latitude: DataRef<f64, ReadOnly>,  // degrees
    longitude: DataRef<f64, ReadOnly>, // ...

    indicated_airspeed: DataRef<f32, ReadOnly>, // knot indicated airspeed
    barometer_inhg: DataRef<f32, ReadOnly>,

    temperature_ambient_c: DataRef<f32, ReadOnly>, // temp outside the aircraft
    //temperature_le_c: DataRef<f32, ReadOnly>,      // temp at the leading edge of the wing
    air_density: DataRef<f32, ReadOnly>, // kg / m^3

    // Buffers for bidirectional communication
    incoming: Output<BufferedControlData>,
    outgoing: Input<BufferedFlightData>,

    fl: FlightLoop,
    ser: Arc<Mutex<Option<serial::SystemPort>>>,

    // latency measurement
    latencies: [Duration; NUM_LATENCY_MEASUREMENTS],
    num_latencies: isize,
    last_time: SystemTime,

    // physics engine rate update measurement
    time_start: SystemTime,
    cycle_count: usize,
}

impl FFSim {
    pub fn get_data(&self, time: SystemTime) -> BufferedFlightData {
        // Throttle: we are only interested in first value
        let mut throttle_buf: [f32; 4] = [0.0; 4];
        self.throttle.get(&mut throttle_buf);

        let mut ret = BufferedFlightData {
            roll_rate: self.roll_rate.get(),
            pitch_rate: self.pitch_rate.get(),
            yaw_rate: self.yaw_rate.get(),
            true_theta: self.true_theta.get(),
            true_phi: self.true_phi.get(),
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
            time,
        };

        self.plane_orientation_quaternion.get(&mut ret.plane_orientation_quaternion);

        ret
    }
}

impl Plugin for FFSim {
    type StartErr = FindError;
    fn start() -> Result<Self, Self::StartErr> {
        /* Initialize triple buffers */
        let (incoming_send, incoming_recv)
            = TripleBuffer::new(BufferedControlData::new()).split();
        let (outgoing_send, outgoing_recv)
            = TripleBuffer::new(BufferedFlightData::new()).split();

        let ser: Arc<Mutex<Option<serial::SystemPort>>> = Arc::new(Mutex::new(None));

        /* Get handles to datarefs */
        let mut plugin = FFSim {
            //override_flightcontrol: DataRef::find("sim/operation/override/override_flightcontrol")?.writeable()?,
            override_control_surfaces: DataRef::find("sim/operation/override/override_control_surfaces")?.writeable()?,
            override_throttles: DataRef::find("sim/operation/override/override_throttles")?.writeable()?,

            // XXX: These are based on the Cessna Skyhawk. For other planes you may need to
            //      change which datarefs are used to move the control surfaces!
            //
            // Also while we're on the subject. A name like hstab1_elv1def means:
            //  * The control surfaces is attached to the horizontal (h) stabilizer (stab)
            //  * The control surface moves when the elevator (elv) command is sent from the yoke.
            rudder: DataRef::find("sim/flightmodel/controls/vstab1_rud1def")?.writeable()?,
            left_aileron: DataRef::find("sim/flightmodel/controls/wing1l_ail1def")?.writeable()?,
            right_aileron: DataRef::find("sim/flightmodel/controls/wing1r_ail1def")?.writeable()?,
            elevator1: DataRef::find("sim/flightmodel/controls/hstab1_elv1def")?.writeable()?,
            elevator2: DataRef::find("sim/flightmodel/controls/hstab2_elv1def")?.writeable()?,

            throttle: DataRef::find("sim/flightmodel/engine/ENGN_thro_use")?.writeable()?,

            // append "rad" to the end of the names to get these in radians
            roll_rate: DataRef::find("sim/flightmodel/position/P")?,
            pitch_rate: DataRef::find("sim/flightmodel/position/Q")?,
            yaw_rate: DataRef::find("sim/flightmodel/position/R")?,

            true_theta: DataRef::find("sim/flightmodel/position/true_theta")?,
            true_phi: DataRef::find("sim/flightmodel/position/true_phi")?,
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
            //temperature_le_c: DataRef::find("sim/weather/temperature_le_c")?,
            air_density: DataRef::find("sim/physics/rho_sea_level")?,

            incoming: incoming_recv,
            outgoing: outgoing_send,

            /* Read control inputs and write flight data to the buffers every flight cycle */
            fl: FlightLoop::new(flight_loop),

            ser: ser.clone(),

            latencies: [Duration::from_millis(0); NUM_LATENCY_MEASUREMENTS],
            num_latencies: - (SACRIFICE_LATENCY_MEASUREMENTS as isize),
            last_time: UNIX_EPOCH,

            time_start: UNIX_EPOCH,
            cycle_count: 0,
        };

        //plugin.override_flightcontrol.set(true);
        plugin.override_control_surfaces.set(true);
        plugin.override_throttles.set(true);

        STOP_THREADS.store(false, Ordering::SeqCst);

        /* Thread to send flight data to controller */
        let ser_tmp1 = ser.clone();
        thread::spawn(move|| comm::send_flight_data_thread(outgoing_recv, ser_tmp1));

        /* Thread to receive controller inputs */
        let ser_tmp2 = ser.clone();
        thread::spawn(move|| comm::recv_control_data_thread(incoming_send, ser_tmp2));

        plugin.fl.schedule_immediate();

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
        self.fl.schedule_immediate();

        //plugin.override_flightcontrol.set(true);
        self.override_control_surfaces.set(true);
        self.override_throttles.set(true);
    }
    
    fn disable(&mut self) {
        self.fl.deactivate();

        //self.override_flightcontrol.set(false);
        self.override_control_surfaces.set(false);
        self.override_throttles.set(false);
    }
    
    fn stop(&mut self) {
        self.fl.deactivate();

        //self.override_flightcontrol.set(false);
        self.override_control_surfaces.set(false);
        self.override_throttles.set(false);

        STOP_THREADS.store(true, Ordering::SeqCst);

        match self.ser.lock().unwrap().as_mut() {
            Some(port) => port.close(),
            None => (),
        };
    }
}

xplane_plugin!(FFSim);
