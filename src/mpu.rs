use defmt::*;
use embassy_executor;
use embassy_stm32::i2c::{I2c, mode::Master};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use mpu6050::*;

#[embassy_executor::task]
pub async fn mpu_task(i2c: I2c<'static, Async, Master>, mut led: Output<'static>) {

    info!("Connecting to MPU6050...");

    // Initialize MPU6050 (default address 0x68)
    let mut mpu = Mpu6050::new(i2c);
    
    // Wait for sensor to stabilize
    Timer::after(Duration::from_millis(100)).await;
    
    // Initialize the sensor
    let mut delay = embassy_time::Delay;
    match mpu.init(&mut delay) {
        Ok(_) => {
            info!("MPU6050 initialized successfully!");
            // Blink LED rapidly to show success
            for _ in 0..6 {
                led.set_low();
                Timer::after(Duration::from_millis(50)).await;
                led.set_high();
                Timer::after(Duration::from_millis(50)).await;
            }
        }
        Err(_) => {
            error!("Failed to initialize MPU6050!");
            // Blink LED slowly to show error
            loop {
                led.set_low();
                Timer::after(Duration::from_millis(500)).await;
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }

    info!("Starting main loop...");

    // Main loop - read sensor data
    loop {
        led.toggle();

        // Read accelerometer data (in g's)
        match mpu.get_acc() {
            Ok(acc) => {
                info!(
                    "Accel - X: {}, Y: {}, Z: {}",
                    acc.x, acc.y, acc.z
                );
            }
            Err(_) => warn!("Failed to read accelerometer"),
        }

        // Read gyroscope data (in degrees/sec)
        match mpu.get_gyro() {
            Ok(gyro) => {
                info!(
                    "Gyro  - X: {}, Y: {}, Z: {}",
                    gyro.x, gyro.y, gyro.z
                );
            }
            Err(_) => warn!("Failed to read gyroscope"),
        }

        // Read temperature (in Celsius)
        match mpu.get_temp() {
            Ok(temp) => {
                info!("Temp: {} °C", temp);
            }
            Err(_) => warn!("Failed to read temperature"),
        }

        // Read roll and pitch angles
        match mpu.get_acc_angles() {
            Ok(angles) => {
                info!(
                    "Angles - Roll: {}, Pitch: {}",
                    angles.x, angles.y
                );
            }
            Err(_) => warn!("Failed to read angles"),
        }

        info!("---");

        // Wait before next reading
        Timer::after(Duration::from_millis(500)).await;
    }
}