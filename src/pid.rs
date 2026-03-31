use nalgebra::{Vector3, UnitQuaternion};
use embassy_time::Instant;
use nalgebra::Vector4;

// Direct PID gains — outputs are motor fractions, inputs are rad / rad/s.
// Tune empirically: start low and increase.
const KP_ROLL_PITCH: f32 = 0.07;
const KI_ROLL_PITCH: f32 = 0.0;
const KD_ROLL_PITCH: f32 = 0.01;

// Yaw PD gains
const KP_YAW: f32 = 0.0;
const KD_YAW: f32 = 0.0;

// Integral anti-windup: max motor fraction contribution from I-term.
const INTEGRAL_LIMIT: f32 = 0.3;

// Low-pass filter cutoff for the gyro rate used in the D-term (Hz).
const GYRO_LPF_CUTOFF_HZ: f32 = 20.0;


pub struct PID {
    integral: Vector3<f32>,
    omega_filtered: Vector3<f32>,
    last_update: Option<Instant>,
}

impl PID {
    pub fn new() -> Self {
        PID {
            integral: Vector3::zeros(),
            omega_filtered: Vector3::zeros(),
            last_update: None,
        }
    }

    // Returns roll/pitch/yaw correction directly as motor fractions [-1, 1].
    // error = actual - desired (positive when overshooting).
    fn compute_correction(&mut self, error: Vector3<f32>, omega: Vector3<f32>) -> Vector3<f32> {
        let dt = match self.last_update {
            None => 0.0,
            Some(t) => t.elapsed().as_micros() as f32 / 1_000_000.0,
        };

        // First-order low-pass filter on gyro rate (D-term only).
        // tau = 1 / (2π * fc);  alpha = tau / (tau + dt)
        let tau_lpf = 1.0 / (2.0 * core::f32::consts::PI * GYRO_LPF_CUTOFF_HZ);
        let alpha = if dt > 0.0 { tau_lpf / (tau_lpf + dt) } else { 1.0 };
        self.omega_filtered = alpha * self.omega_filtered + (1.0 - alpha) * omega;
        let omega_d = self.omega_filtered;

        // info!("Error x: {}, error y: {}", error.x, error.y);

        // Integral for roll and pitch only; clamp to prevent windup.
        self.integral.x = (self.integral.x + error.x * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        self.integral.y = (self.integral.y + error.y * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        self.last_update = Some(Instant::now());

        Vector3::new(
            -(KP_ROLL_PITCH * error.x + KI_ROLL_PITCH * self.integral.x + KD_ROLL_PITCH * omega_d.x),
            -(KP_ROLL_PITCH * error.y + KI_ROLL_PITCH * self.integral.y + KD_ROLL_PITCH * omega_d.y),
            -(KP_YAW        * error.z                                    + KD_YAW        * omega_d.z),
        )
    }

    /// Returns per-motor normalized commands in [0.0, 1.0].
    ///
    /// PID outputs motor fractions directly — no inertia model needed.
    /// The caller maps 0.0 → 1000 µs and 1.0 → 2000 µs.
    pub fn control(
        &mut self,
        attitude: UnitQuaternion<f32>,
        omega: Vector3<f32>,           // bias-corrected body angular rate (rad/s)
        desired_attitude: UnitQuaternion<f32>,
        throttle: f32,  // 0.0–1.0, fraction of total available thrust
    ) -> Vector4<f32> {
        let correction = self.compute_correction(
            (desired_attitude.inverse() * attitude).scaled_axis(),
            omega,
        );

        let roll  = correction.x;
        let pitch = correction.y;
        // Yaw excluded: no magnetometer means EKF yaw drifts freely.
        let yaw   = 0.0_f32;

        // X-frame motor mixing.
        // Motor layout (top view):  M2(FL) M1(FR)
        //                           M3(BL) M4(BR)
        // Spin directions:          CW     CCW
        //                           CCW     CW
        let m1 = (throttle - roll + pitch - yaw).clamp(0.0, 0.6);
        let m2 = (throttle + roll + pitch + yaw).clamp(0.0, 0.6);
        let m3 = (throttle + roll - pitch - yaw).clamp(0.0, 0.6);
        let m4 = (throttle - roll - pitch + yaw).clamp(0.0, 0.6);

        // let m1 = (pitch).clamp(0.0, 0.5);
        // let m2 = (pitch).clamp(0.0, 0.5);
        // let m3 = (-pitch).clamp(0.0, 0.5);
        // let m4 = (-pitch).clamp(0.0, 0.5);

        // let m1 = (-roll).clamp(0.0, 0.5);
        // let m2 = (roll).clamp(0.0, 0.5);
        // let m3 = (roll).clamp(0.0, 0.5);
        // let m4 = (-roll).clamp(0.0, 0.5);

        Vector4::new(m1, m2, m3, m4)
    }
}
