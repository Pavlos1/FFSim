#[repr(C)]
pub struct ControlData {
    // "SYNC" in ASCII. Won't appear in the body of the struct
    // since the 5 leading bits of each field shall be zero.
    sync: [u8; 4],

    // 11 bits each; unsigned so at 0 control surface is down.
    rudder: u16,
    left_aileron: u16,
    right_aileron: u16,
    elevator: u16,

    // also 11 bit unsigned
    throttle: u16,
    _pad: u16, // better to be explicit

    // Sum of bytes between sync and checksum, modulo 4 bytes, all bits flipped (1's complement)
    checksum: u32,
}