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
}