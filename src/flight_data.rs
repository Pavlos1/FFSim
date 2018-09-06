use std::f32::consts::PI;

use super::BufferedFlightData;

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