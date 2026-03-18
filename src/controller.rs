use defmt::*;
use rust_ekf::EKF;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use embassy_time::{Instant, Duration, Timer};
use embassy_stm32::gpio::Output;

use crate::motors::MotorsCmd;
use crate::pid::PID;
use crate::mpu::MpuRcv;

const GRAVITY: f32 = 9.81;

#[embassy_executor::task]
pub async fn control_loop(
    mpu_rcv: & 'static MpuRcv,
    cmd_motors: & 'static MotorsCmd,
    mut led: Output<'static>,
) -> ! {

    // Collect samples at rest to estimate gyro bias and initial orientation.
    const CALIB_SAMPLES: usize = 200;
    let mut gyro_sum = [0.0f32; 3];
    let mut accel_sum = [0.0f32; 3];
    for _ in 0..CALIB_SAMPLES {
        let s = mpu_rcv.receive().await;
        gyro_sum[0] += -s.gyro[0];
        gyro_sum[1] +=  s.gyro[1];
        gyro_sum[2] += -s.gyro[2];
        accel_sum[0] += -s.acc[0];
        accel_sum[1] +=  s.acc[1];
        accel_sum[2] += -s.acc[2];
    }
    let n = CALIB_SAMPLES as f32;
    let gyro_bias = [gyro_sum[0] / n, gyro_sum[1] / n, gyro_sum[2] / n];
    let init_accel = [accel_sum[0] / n * GRAVITY, accel_sum[1] / n * GRAVITY, accel_sum[2] / n * GRAVITY];
    info!("Calibration done. Gyro bias: [{} {} {}]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    let mut ekf = EKF::new(Some(init_accel), Some(gyro_bias));
    let mut pid = PID::new();

    let mut loop_count = 0;
    let mut last_update = Instant::now();

    let mut stop = false;

    loop {

        if loop_count % 100 == 0 {
            led.toggle();
        }

        let start_time = Instant::now();
        let dt = start_time - last_update;
        let dt_secs = dt.as_micros() as f32 / 1000000.0;

        let mpu_data = mpu_rcv.receive().await;

        // Get gyro data
        let gyro_data = [-mpu_data.gyro[0], mpu_data.gyro[1], -mpu_data.gyro[2]];
        // info!("Gyro data: {:?}", gyro_data);

        // Prediction phase of the EKF
        ekf.predict(gyro_data, dt_secs);

        // Get acceleration data — sensor returns g, EKF expects m/s²
        let accel_data = [-mpu_data.acc[0] * GRAVITY, mpu_data.acc[1] * GRAVITY, -mpu_data.acc[2] * GRAVITY];
        // info!("Acc data: {:?}", accel_data);

        // Update phase of the EKF
        ekf.update(accel_data);
        last_update = start_time;

        // Get the updated state of the EKF
        let state = ekf.get_state();
        let attitude_q = UnitQuaternion::from_quaternion(Quaternion::new(state[0], state[1], state[2], state[3]));
        let (roll, pitch, yaw) = attitude_q.euler_angles();
        let to_deg = 180.0 / core::f32::consts::PI;
        
        // if loop_count % 100 == 0 {
        //     info!(
        //         "roll={} pitch={} yaw={} | bias=[{} {} {}]",
        //         roll * to_deg, pitch * to_deg, yaw * to_deg,
        //         state[4], state[5], state[6]
        //     );
        // }

        let attitude = UnitQuaternion::from_quaternion(Quaternion::new(state[0], state[1], state[2], state[3]));
        // Bias-corrected body angular rate (rad/s)
        let omega = Vector3::new(gyro_data[0] - state[4], gyro_data[1] - state[5], gyro_data[2] - state[6]);
        let desired_attitude = UnitQuaternion::from_euler_angles(0., 0., 0.0);
        // Hover throttle: fraction of total available thrust to hold altitude.
        // Start low (~0.3) and increase slowly while holding the drone until
        // it becomes light. Theoretical value: mg / (4 * THRUST_MAX_PER_MOTOR).
        let throttle = 0.38_f32;

        if state[4].abs() > 2. || state[5].abs() > 2. {
            stop = true;
        }

        let power_motors = match stop {
            false => pid.control(attitude, omega, desired_attitude, throttle),
            true => Vector4::new(0.0,0.0,0.0,0.0),
        };
        // let power_motors = Vector4::new(0.3,0.3,0.3,0.3);

        if loop_count % 100 == 0 {
            info!("Power motors: {:?}", Debug2Format(&power_motors));
        }

        cmd_motors.send(power_motors).await;

        // info!("Elapsed time end: {} us", start_time.elapsed().as_micros());

        loop_count += 1;

    }
}
