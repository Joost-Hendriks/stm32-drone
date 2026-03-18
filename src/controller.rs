use defmt::*;
use rust_ekf::EKF;
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use embassy_time::{Instant, Duration, Timer};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{I2c, mode::Master};
use embassy_stm32::mode::Async;
use mpu6050::*;
use embassy_stm32::exti::ExtiInput;

use crate::motors::{Motors, set_power};
use crate::pid::PID;
use crate::mpu::read_mpu;
use crate::receiver::pwm_receiver_channel;

// Cycle time needs to be longer than total loop time 
// to give room for PWM signal.
const CYCLE_TIME_MS: u64 = 10;
const GRAVITY: f32 = 9.81;

#[embassy_executor::task]
pub async fn control_loop(
    mpu: &'static mut Mpu6050<I2c<'static, Async, Master>>,
    motors: &'static mut Motors,
    mut led: Output<'static>,
    channels: [&'static mut ExtiInput<'static>; 6]
) -> ! {

    // Collect samples at rest to estimate gyro bias and initial orientation.
    const CALIB_SAMPLES: usize = 200;
    let mut gyro_sum = [0.0f32; 3];
    let mut accel_sum = [0.0f32; 3];
    for _ in 0..CALIB_SAMPLES {
        let s = read_mpu(mpu).await;
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
    let mut channel_n = 0;
    // Stored PWM pulse widths in µs; initialized to neutral (1500µs) except thrust (1000µs).
    // Layout: [0]=Yaw, [1]=Pitch, [2]=Thrust, [3]=Roll, [4]=Var1, [5]=Var2
    let mut channels_us: [u64; 6] = [1500, 1500, 1000, 1500, 1500, 1500];

    // Max desired tilt angle in radians (±30°)
    const MAX_TILT_RAD: f32 = 30.0 * core::f32::consts::PI / 180.0;
    // Throttle range mapped from full-stick (1000–2000µs) to [0.0, MAX_THROTTLE]
    const MAX_THROTTLE: f32 = 0.6;

    loop {

        if loop_count % 100 == 0 {
            led.toggle();
        }

        let start_time = Instant::now();
        let dt = start_time - last_update;
        let dt_secs = dt.as_micros() as f32 / 1000000.0;

        let mpu_data = read_mpu(mpu).await;

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
        
        if loop_count % 100 == 0 {
            info!(
                "roll={} pitch={} yaw={} | bias=[{} {} {}]",
                roll * to_deg, pitch * to_deg, yaw * to_deg,
                state[4], state[5], state[6]
            );
        }

        let attitude = UnitQuaternion::from_quaternion(Quaternion::new(state[0], state[1], state[2], state[3]));
        // Bias-corrected body angular rate (rad/s)
        let omega = Vector3::new(gyro_data[0] - state[4], gyro_data[1] - state[5], gyro_data[2] - state[6]);

        // Map PWM pulse widths to desired attitude angles.
        // Channels are centred at 1500µs; ±500µs maps to ±MAX_TILT_RAD.
        let norm = |us: u64| (us as f32 - 1500.0) / 500.0; // [-1, 1]
        let desired_roll  =  norm(channels_us[3]) * MAX_TILT_RAD; // CH4
        let desired_pitch =  norm(channels_us[1]) * MAX_TILT_RAD; // CH2
        let desired_yaw   =  norm(channels_us[0]) * core::f32::consts::PI; // CH1 full rotation
        // info!("Desired roll: {}", desired_roll);

        let desired_attitude = UnitQuaternion::from_euler_angles(desired_roll, desired_pitch, desired_yaw);

        // Map thrust channel (CH3) from [1000, 2000]µs to [0.0, MAX_THROTTLE].
        let throttle = ((channels_us[2] as f32 - 1000.0) / 1000.0).clamp(0.0, 1.0) * MAX_THROTTLE;

        if state[4].abs() > 2. || state[5].abs() > 2. {
            stop = true;
        }

        let power_motors = match stop {
            false => pid.control(attitude, omega, desired_attitude, throttle),
            true => Vector4::new(0.0,0.0,0.0,0.0),
        };
        // let power_motors = Vector4::new(0.3,0.0,0.0,0.0);

        // if loop_count % 100 == 0 {
        //     info!("Power motors: {:?}", Debug2Format(&power_motors));
        // }

        set_power(motors, power_motors).await;

        // info!("Elapsed time end: {} us", start_time.elapsed().as_micros());
        if loop_count % 10 == 0 {
            channels_us[channel_n] = pwm_receiver_channel(channels[channel_n], channel_n).await;
            channel_n = (channel_n + 1) % 6;
        }

        // Timer::at(start_time + Duration::from_millis(CYCLE_TIME_MS)).await;
        loop_count += 1;

    }
}
