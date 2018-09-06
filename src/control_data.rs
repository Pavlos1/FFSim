#[repr(C)]
pub struct ControlData {
    // 11 bits each; unsigned so at 0 control surface is down.
    rudder: u16,
    left_aileron: u16,
    right_aileron: u16,
    elevator: u16,

    // also 11 bit unsigned
    throttle: u16,
}