use nalgebra::{Matrix3, Vector3, UnitQuaternion};
use embassy_time::Instant;
use nalgebra::Vector4;

// Inertia tensor (kg·m²), matching drone-modelling/main.py
const IXX: f32 = 0.014;
const IYY: f32 = 0.014;
const IZZ: f32 = 0.028;

// Roll / pitch PID gains, matching drone-modelling/main.py
const KP_ROLL_PITCH: f32 = 2.0;
const KI_ROLL_PITCH: f32 = 0.0;
const KD_ROLL_PITCH: f32 = 0.5;

// Yaw PD gains (no integral term), matching drone-modelling/main.py
const KP_YAW: f32 = 5.0;
const KD_YAW: f32 = 1.0;

// Motor mixing geometry (450mm X-frame)
const ARM_LENGTH: f32 = 0.225;          // m — centre to motor
const THRUST_MAX_PER_MOTOR: f32 = 7.85; // N

// Maximum achievable roll / pitch torque:
//   one arm pair at full differential → L * 2 * F_max
const MAX_TORQUE_RP: f32 = ARM_LENGTH * THRUST_MAX_PER_MOTOR * 2.0;

// Integral anti-windup clamp: limits the integral so its contribution
// (Ki * integral) never exceeds the maximum achievable torque.
const INTEGRAL_LIMIT: f32 = MAX_TORQUE_RP / KI_ROLL_PITCH;

// Drag/thrust ratio for yaw (~2 % is typical for 5-inch props).
// Increase if yaw response feels sluggish; decrease if it oscillates.
const K_YAW: f32 = 0.02;                // m
const MAX_TORQUE_YAW: f32 = K_YAW * THRUST_MAX_PER_MOTOR * 2.0;

pub struct PID {
    integral: Vector3<f32>,
    last_update: Option<Instant>,
}

impl PID {
    pub fn new() -> Self {
        PID {
            integral: Vector3::zeros(),
            last_update: None,
        }
    }

    // Returns commanded torque [N·m] such that I*alpha = tau - omega x (I*omega).
    // error = actual - desired (positive when over-shooting).
    fn compute_torque(&mut self, error: Vector3<f32>, omega: Vector3<f32>) -> Vector3<f32> {
        let dt = match self.last_update {
            // First call: no previous timestamp, skip integral accumulation.
            None => 0.0,
            // Cap at 2× the expected cycle time to prevent a large integral
            // spike if the loop is ever delayed (e.g. channel back-pressure).
            Some(t) => (t.elapsed().as_micros() as f32 / 1_000_000.0).min(0.02),
        };

        // Accumulate integral for roll and pitch only; yaw is PD.
        // Clamp to prevent windup when the drone is held on the ground or
        // the output is saturated.
        self.integral.x = (self.integral.x + error.x * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        self.integral.y = (self.integral.y + error.y * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        let i = Matrix3::from_diagonal(&Vector3::new(IXX, IYY, IZZ));

        // Desired angular acceleration per axis.
        // Negated because error = actual - desired, so correction = -Kp*error.
        let alpha_des = Vector3::new(
            -(KP_ROLL_PITCH * error.x + KI_ROLL_PITCH * self.integral.x + KD_ROLL_PITCH * omega.x),
            -(KP_ROLL_PITCH * error.y + KI_ROLL_PITCH * self.integral.y + KD_ROLL_PITCH * omega.y),
            -(KP_YAW        * error.z                                    + KD_YAW        * omega.z),
        );

        // Full inverse-dynamics torque (Euler's rotation equation):
        //   tau = I * alpha_des + omega x (I * omega)
        let torque = i * alpha_des + omega.cross(&(i * omega));

        self.last_update = Some(Instant::now());

        torque
    }

    /// Returns per-motor normalized commands in [0.0, 1.0].
    ///
    /// Conversion chain:
    ///   torques [N·m]
    ///   → normalize by max achievable torque  (→ dimensionless, ≈ -1..1)
    ///   → motor mixing (X-frame geometry)     (→ per-motor fraction, 0..1)
    ///
    /// The caller (motors_task) maps 0.0 → 1000 µs and 1.0 → 2000 µs.
    pub fn control(
        &mut self,
        attitude: UnitQuaternion<f32>,
        omega: Vector3<f32>,           // bias-corrected body angular rate (rad/s)
        desired_attitude: UnitQuaternion<f32>,
        throttle: f32,  // 0.0–1.0, fraction of total available thrust
    ) -> Vector4<f32> {
        let tau = self.compute_torque(
            attitude.scaled_axis() - desired_attitude.scaled_axis(),
            omega,
        );

        // Normalize torques to [-1, 1] using the physical limits of the frame.
        let roll  = tau.x / MAX_TORQUE_RP;
        let pitch = tau.y / MAX_TORQUE_RP;
        let yaw   = tau.z / MAX_TORQUE_YAW;

        // X-frame motor mixing.
        // Motor layout (top view):  M1(FR) M2(FL)
        //                           M4(BR) M3(BL)
        // Spin directions:          CW     CCW
        //                           CCW    CW
        let m1 = (throttle - roll + pitch - yaw).clamp(0.0, 1.0);
        let m2 = (throttle + roll + pitch + yaw).clamp(0.0, 1.0);
        let m3 = (throttle + roll - pitch - yaw).clamp(0.0, 1.0);
        let m4 = (throttle - roll - pitch + yaw).clamp(0.0, 1.0);

        Vector4::new(m1, m2, m3, m4)
    }
}
