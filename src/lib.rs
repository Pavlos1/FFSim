#[macro_use(xplane_plugin)]
extern crate xplm;
use xplm::plugin::{Plugin, PluginInfo};

use xplm::data::borrowed::{DataRef, FindError};
use xplm::data::{ReadOnly, ReadWrite, DataRead, DataReadWrite, ArrayRead, StringRead};

extern crate triple_buffer;

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
    plane_orientation_quaternion: DataRef<[f32], ReadOnly>, // Remember to negate 4th component

    latitude: DataRef<f64, ReadOnly>,  // degrees
    longitude: DataRef<f64, ReadOnly>, // ...

    indicated_airspeed: DataRef<f32, ReadOnly>,
    altitude: DataRef<f32, ReadOnly>,
}

impl Plugin for FFSim {
    type StartErr = FindError;
    fn start() -> Result<Self, Self::StartErr> {
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
        };

        //plugin.override_flightcontrol.set(true);
        plugin.override_control_surfaces.set(true);
        plugin.override_throttles.set(true);

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
    }
}

xplane_plugin!(FFSim);
