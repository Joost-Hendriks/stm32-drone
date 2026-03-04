use nalgebra::{Vector3, UnitQuaternion};
use embassy_time::{Instant, Duration};
use nalgebra::Vector4;

pub struct PID {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: Vector3<f64>,
    last_update: Instant,
}

impl PID {
    pub fn new() -> Self {
        PID {
            kp: 0.0,
            ki: 0.0,
            kd: 1.0,
            integral: Vector3::new(0., 0., 0.),
            last_update: Instant::now(),
        }
    }

    fn compute_torque(&mut self, error: Vector3<f64>, angular_rate: Vector3<f64>) -> Vector3<f64> {
        let diff = self.last_update.elapsed();
        let dt = diff.as_micros() as f64;

        self.integral = self.integral + error * dt;
        let torque = -self.kp * error - self.kd * angular_rate - self.ki * self.integral;

        self.last_update = Instant::now();

        torque
    }

    pub fn control(
        &mut self,
        attitude: UnitQuaternion<f64>,
        angular_rate: UnitQuaternion<f64>,
        desired_attitude: UnitQuaternion<f64>,
        throttle: f64,  // 0.0–1.0
    ) -> Vector4<u16> {
        let torque = self.compute_torque(
            attitude.scaled_axis() - desired_attitude.scaled_axis(),
            angular_rate.scaled_axis(),
        );

        let roll  = torque.x;
        let pitch = torque.y;
        let yaw   = torque.z;

        // Motor mixing (X-frame)
        let m1 = throttle - roll + pitch - yaw;
        let m2 = throttle + roll + pitch + yaw;
        let m3 = throttle + roll - pitch - yaw;
        let m4 = throttle - roll - pitch + yaw;

        // Clamp and scale to PWM range (e.g. 1000–2000 µs)
        let scale = |v: f64| (v.clamp(0.0, 1.0) * 1000.0 + 1000.0) as u16;

        Vector4::new(scale(m1), scale(m2), scale(m3), scale(m4))
    }
}