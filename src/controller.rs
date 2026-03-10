use defmt::*;
use rust_ekf::EKF;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use embassy_time::{Instant, Duration, Timer};
use embassy_stm32::gpio::Output;

use crate::motors::MotorsCmd;
use crate::pid::PID;
use crate::mpu::MpuRcv;

// Cycle time in seconds
const CYCLE_TIME: f64 = 0.01;
const GRAVITY: f64 = 9.81;

#[embassy_executor::task]
pub async fn control_loop(
    mpu_rcv: & 'static MpuRcv, 
    cmd_motors: & 'static MotorsCmd,
    mut led: Output<'static>,
) -> ! {

    // Seed the EKF with the first real accelerometer reading so it starts
    // close to the true attitude instead of the identity quaternion.
    let first_sample = mpu_rcv.receive().await;
    let init_accel = [
        first_sample.acc[1] as f64,
        first_sample.acc[0] as f64,
        -first_sample.acc[2] as f64,
    ];
    let mut ekf = EKF::new(Some(init_accel));
    let mut pid = PID::new();

    let mut loop_count = 0;

    loop {

        if loop_count % 10 == 0 {
            led.toggle();
        }

        let start_time = Instant::now();

        let mpu_data = mpu_rcv.receive().await;

        // Get gyro data
        let gyro_data = [mpu_data.gyro[1] as f64, mpu_data.gyro[0] as f64, -mpu_data.gyro[2] as f64];
        // let gyro_data = [0 as f64, 0 as f64, 0 as f64];
        // info!("Gyro data: {:?}", gyro_data[2]);

        // Prediction phase of the EKF
        ekf.predict(gyro_data, CYCLE_TIME);

        // Get acceleration data — sensor returns g, EKF expects m/s²
        let accel_data = [mpu_data.acc[1] as f64 * GRAVITY, mpu_data.acc[0] as f64 * GRAVITY, -mpu_data.acc[2] as f64 * GRAVITY];
        // let accel_data = [0 as f64, 0 as f64, 0 as f64];
        // info!("Acc data: {:?}", accel_data[2]);

        // Update phase of the EKF
        ekf.update(accel_data);

        // Get the updated state of the EKF
        let state = ekf.get_state();
        // info!("Updated State Vector: {:?}", Debug2Format(&state));

        let attitude = UnitQuaternion::from_quaternion(Quaternion::new(state[0], state[1], state[2], state[3]));
        // Bias-corrected body angular rate (rad/s)
        let omega = Vector3::new(gyro_data[0] - state[4], gyro_data[1] - state[5], gyro_data[2] - state[6]);
        let desired_attitude = UnitQuaternion::from_euler_angles(0., 0., 0.0);
        // Hover throttle: fraction of total available thrust to hold altitude.
        // Tune this to mg / (4 * THRUST_MAX_PER_MOTOR). Must be in [0.0, 1.0].
        let throttle = 1.;

        let power_motors = pid.control(attitude, omega, desired_attitude, throttle);

        // let torque = Vector4::new(0., 0., 0., 0.);
        cmd_motors.send(power_motors).await;

        Timer::at(start_time + Duration::from_millis((CYCLE_TIME * 1000.0) as u64)).await;

        loop_count += 1;

    }
}