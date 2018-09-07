use std::mem::transmute;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct ControlData {
    // "SYNC" in ASCII. Won't appear in the body of the struct
    // since the 5 leading bits of each field shall be zero.
    sync: [u8; 4],

    // 11 bits each; unsigned so at 0 control surface is down.
    pub rudder: u16,
    pub left_aileron: u16,
    pub right_aileron: u16,
    pub elevator: u16,

    // also 11 bit unsigned
    pub throttle: u16,
    _pad: u16, // better to be explicit

    // Sum of bytes between sync and checksum, modulo 4 bytes, all bits flipped (1's complement)
    checksum: u32,
}

impl ControlData {
    pub fn verify(&self) -> bool {
        let raw_bytes: [u8; 20] = unsafe { transmute(*self) };
        (!(raw_bytes[4 .. 16].iter().fold(0u32, |sum, val| sum + (*val as u32)))
            == self.checksum) && (raw_bytes[.. 4] == *("SYNC".as_bytes()))
    }
}