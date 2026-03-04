use defmt::*;
use rust_ekf::EKF;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use embassy_time::{Instant, Duration, Timer};

use crate::motors::MotorsCmd;
use crate::pid::PID;
use crate::mpu::MpuRcv;

// Cycle time in milliseconds
const CYCLE_TIME: f64 = 5.;

#[embassy_executor::task]
pub async fn control_loop(mpu_rcv: & 'static MpuRcv, cmd_motors: & 'static MotorsCmd) -> ! {

    let mut ekf = EKF::new(None);
    let mut pid = PID::new();

    loop {

        let start_time = Instant::now();

        let mpu_data = mpu_rcv.receive().await;

        // Get gyro data
        let gyro_data = [mpu_data.gyro[0] as f64, mpu_data.gyro[1] as f64, mpu_data.gyro[2] as f64];

        // Prediction phase of the EKF
        ekf.predict(gyro_data, CYCLE_TIME);

        // Get acceleration data
        let accel_data = [mpu_data.acc[0] as f64, mpu_data.acc[1] as f64, mpu_data.acc[2] as f64];

        // Update phase of the EKF
        ekf.update(accel_data);

        // Get the updated state of the EKF
        let state = ekf.get_state();
        // info!("Updated State Vector: {:?}", Debug2Format(&state));

        let attitude = UnitQuaternion::from_quaternion(Quaternion::new(state[0], state[1], state[2], state[3]));
        let omega = Vector3::new(gyro_data[0] - state[4], gyro_data[1] - state[5], gyro_data[2] - state[6]);
        let angular_rate = UnitQuaternion::from_scaled_axis(omega);
        let desired_attitude = UnitQuaternion::from_euler_angles(0., 0., 0.0);
        let throttle = 0.;

        let torque = pid.control(attitude, angular_rate, desired_attitude, throttle);

        cmd_motors.send(torque).await;

        Timer::at(start_time + Duration::from_millis(CYCLE_TIME as u64)).await;

    }
}