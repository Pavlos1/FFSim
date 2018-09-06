#[macro_use(xplane_plugin)]
extern crate xplm;
use xplm::plugin::{Plugin, PluginInfo};

use xplm::data::borrowed::{DataRef, FindError};
use xplm::data::{ReadOnly, ReadWrite, DataRead, DataReadWrite, ArrayRead, ArrayReadWrite};
use xplm::flight_loop::{FlightLoop, LoopState};
use triple_buffer::{TripleBuffer, Input, Output};
use std::{thread, time};
use std::f32::consts::PI;

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
    barometer_inhg: f32,

    ambient_temp: f32,
    air_density: f32,
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
            ambient_temp: 0.0,
            barometer_inhg: 0.0,
            air_density: 0.0,
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

#[repr(C)]
struct FlightData {
    // lsm6dsm: Outputs are in 2's complement, 16 bits
    // Units: X milli-dps / least-significant-bit,
    //        depending on Full Scale representation.
    //        (See data sheet, we're using PM 2000 dps)
    roll_rate: i16,
    pitch_rate: i16,
    yaw_rate: i16,

    // Sim. to above, same datasheet, PM 8G
    lin_acc_x: i16,
    lin_acc_y: i16,
    lin_acc_z: i16,

    // en.DM00075867, sim to above .. PM 4 gauss
    mag_x: i16,
    mag_y: i16,
    mag_z: i16,
    temp: i16, // can also get from barometer, both 16 bits, also both _ambient_ probably

    // lps25hb
    barometer: u32, // 24 bits, 4K LSB/hPa, abs. range 260-1260 hPa
                    // XXX: Technically this is 2's complement 24-bits,
                    // but on default settings (i.e. if you don't mess
                    // with the reference pressure) a negative output
                    // should be impossible.

    // Sensirion_Differential_Pressure_Sensors_SDP3x_Digital_Datasheet
    // 60 or 240 Pa/LSB for 31 and 32 resp. Probably 32.
    airspeed_pressure: i16,

    // GPS in NMEA
    gps: [u8; 82],
}

impl FlightData {
    fn new(bfd: BufferedFlightData) -> Self {
        /* See comments on `FlightData` for info about conversions */
        let angular_rate_conversion: f32 = 1000f32 / 70f32;

        let temperature_conversion: f32 = 256f32;
        let temperature_offset: f32 = 0f32; // XXX: configurable via IMU registers, deg C, PM 15

        // I would support nuking the U.S. if it means we get rid of imperial units,
        let inhg_to_hpa: f32 = 338.639f32;
        let barometer_conversion: f32 = inhg_to_hpa * 4096f32;

        let knots_to_ms: f32 = 0.5144447f32;
        let kias_to_pa = |kias: f32| -> f32 {
            (bfd.air_density * (kias * knots_to_ms) * (kias * knots_to_ms)) / 2f32
        };
        let airspeed_pressure_conversion: f32 = 1f32 / 240f32;

        // Polar coordinate angles of B field vector relative to aircraft
        // (negated since theta/psi were aircraft relative to magnetic field)
        // I would also support bombing the engineering building to get rid of angles in degrees
        let mag_theta: f32 = - bfd.true_theta * PI / 180f32;
        let mag_psi: f32 = - bfd.mag_psi * PI / 180f32;
        // standard conversion to cartesian coordinates
        let norm_mag_x: f32 = mag_theta.sin() * mag_psi.cos();
        let norm_mag_y: f32 = mag_theta.sin() * mag_psi.cos();
        let norm_mag_z: f32 = mag_theta.cos();
        // this is a lie but I don't think we have actual field strength from the sim
        let mag_field_str: f32 = 0.45f32; // in gauss for ease of conversion
        let mag_field_str_conversion: f32 = 6842f32;

        FlightData {
            roll_rate: (bfd.roll_rate * angular_rate_conversion) as i16,
            pitch_rate: (bfd.pitch_rate * angular_rate_conversion) as i16,
            yaw_rate: (bfd.yaw_rate * angular_rate_conversion) as i16,

            lin_acc_x: 0,
            lin_acc_y: 0,
            lin_acc_z: 0,

            mag_x: (norm_mag_x * mag_field_str * mag_field_str_conversion) as i16,
            mag_y: (norm_mag_y * mag_field_str * mag_field_str_conversion) as i16,
            mag_z: (norm_mag_z * mag_field_str * mag_field_str_conversion) as i16,

            temp: ((bfd.ambient_temp + temperature_offset) * temperature_conversion) as i16,
            barometer: (bfd.barometer_inhg * barometer_conversion) as u32,
            airspeed_pressure: (kias_to_pa(bfd.indicated_airspeed)
                * airspeed_pressure_conversion) as i16,

            gps: [0; 82],
        }
    }
}

#[repr(C)]
struct ControlData {
    // 11 bits each; unsigned so at 0 control surface is down.
    rudder: u16,
    left_aileron: u16,
    right_aileron: u16,
    elevator: u16,

    // also 11 bit unsigned
    throttle: u16,
}

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
