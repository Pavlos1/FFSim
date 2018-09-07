use std::f32::consts::PI;
use std::ops::BitXor;

use super::BufferedFlightData;
use super::Quaternion;

#[repr(C)]
pub struct FlightData {
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
    pub fn new(bfd: BufferedFlightData) -> Self {
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

        // The quaternion is from OpenGL coordinates to the plane's, so
        // we invert (conjugate) it, and then rotate the acceleration
        // in OpenGL coordinates.
        // (Units remain m/s^2 since the quaternion is only a rotation)
        let lin_acc = Quaternion::new([
            bfd.plane_orientation_quaternion[0],
            bfd.plane_orientation_quaternion[1],
            bfd.plane_orientation_quaternion[2],
            bfd.plane_orientation_quaternion[3],
        ]).conj().rotate([bfd.local_ax, bfd.local_ay, bfd.local_az]);
        let acc_conversion: f32 = (1f32 / 9.8f32)  // m/s^2 -> g
            * 1000f32 // g -> mg
            * (1f32 / 0.244f32); // mg -> LSB

        FlightData {
            roll_rate: (bfd.roll_rate * angular_rate_conversion) as i16,
            pitch_rate: (bfd.pitch_rate * angular_rate_conversion) as i16,
            yaw_rate: (bfd.yaw_rate * angular_rate_conversion) as i16,

            lin_acc_x: (lin_acc[0] * acc_conversion) as i16,
            lin_acc_y: (lin_acc[1] * acc_conversion) as i16,
            lin_acc_z: (lin_acc[2] * acc_conversion) as i16,

            mag_x: (norm_mag_x * mag_field_str * mag_field_str_conversion) as i16,
            mag_y: (norm_mag_y * mag_field_str * mag_field_str_conversion) as i16,
            mag_z: (norm_mag_z * mag_field_str * mag_field_str_conversion) as i16,

            temp: ((bfd.ambient_temp + temperature_offset) * temperature_conversion) as i16,
            barometer: (bfd.barometer_inhg * barometer_conversion) as u32,
            airspeed_pressure: (kias_to_pa(bfd.indicated_airspeed)
                * airspeed_pressure_conversion) as i16,

            gps: Self::conv_to_nmea(bfd.latitude, bfd.longitude),
        }
    }

    // XXX: There are other NMEA formats we could send,
    //      but for simplicity we'll just send global
    //      position data.
    fn conv_to_nmea(lat: f64, long: f64) -> [u8; 82] {
        let mut res = String::new();

        // header
        res.push_str("$");
        res.push_str("GL");  // GLORY TO THE MOTHERLAND
        res.push_str("GLL"); // Latitude/Longitude info
        res.push_str(",");

        // latitude
        res.push_str(format!("{:.2}", lat.abs()).as_str()); // abs lat to 2dp
        res.push_str(",");
        // sign according to ISO-6709 (hopefully)
        if lat.is_sign_positive() {
            res.push_str("N");
        } else {
            res.push_str("S");
        }
        res.push_str(",");

        // longitude
        res.push_str(format!("{:.2}", long.abs()).as_str());
        res.push_str(",");
        if long.is_sign_positive() {
            res.push_str("E");
        } else {
            res.push_str("W");
        }

        /* We're not bothering with the time of the fix for now
           since the FPGA and flightsim don't synchronize their
           clocks anyway. */

        // checksum
        res.push_str("*");
        let check: u8 = res[1 .. res.len()-1] // the $ and * aren't part of the checksum
            .as_bytes().iter()
            // checksum is XOR of all elements
            .fold(0u8, |tot, val| tot.bitxor(*val));

        res.push_str(format!("{:02X}", check).as_str()); // format as 2 hex digits

        // CRLF indicates end of string
        res.push_str("\r\n");

        let mut ret: [u8; 82] = [0u8; 82];
        ret.copy_from_slice(res.as_bytes());
        ret
    }
}