use super::ControlData;

#[derive(Copy, Clone, Debug)]
pub struct BufferedControlData {
    pub rudder: f32,
    pub left_aileron: f32,
    pub right_aileron: f32,
    pub elevator: f32,

    pub throttle: f32,
}

impl BufferedControlData {
    pub fn new() -> Self {
        BufferedControlData {
            rudder: 0.0,
            left_aileron: 0.0,
            right_aileron: 0.0,
            elevator: 0.0,
            throttle: 0.0,
        }
    }

    pub fn from_external(cd: ControlData) -> Self {
        assert!(cd.verify());

        let max_deflection_deg: f32 = 15f32; // relative to zero in either direction
        // see comments in ControlData struct
        let control_surface_conversion = |input: u16| -> f32 {
            (input as f32) * ((2f32 * max_deflection_deg) / ((1 << 11) - 1) as f32)
                - max_deflection_deg
        };

        BufferedControlData {
            rudder: control_surface_conversion(cd.rudder),
            left_aileron: control_surface_conversion(cd.left_aileron),
            right_aileron: control_surface_conversion(cd.right_aileron),
            elevator: control_surface_conversion(cd.elevator),

            // throttle output is just [0, 1] so we divide it by the full range
            throttle: (cd.throttle as f32) / (((1 << 11) - 1) as f32),
        }
    }
}