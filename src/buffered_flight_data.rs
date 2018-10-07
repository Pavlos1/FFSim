// See FFSim struct for comments about these values
#[derive(Copy, Clone, Debug)]
pub struct BufferedFlightData {
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,

    pub true_theta: f32,
    pub true_phi: f32,
    pub mag_psi: f32,

    pub local_ax: f32,
    pub local_ay: f32,
    pub local_az: f32,
    pub plane_orientation_quaternion: [f32; 4],

    pub latitude: f64,
    pub longitude: f64,

    pub indicated_airspeed: f32,
    pub barometer_inhg: f32,

    pub ambient_temp: f32,
    pub air_density: f32,
}

impl BufferedFlightData {
    pub fn new() -> Self {
        BufferedFlightData {
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
            true_theta: 0.0,
            true_phi: 0.0,
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