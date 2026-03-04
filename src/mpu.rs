use defmt::*;
use embassy_stm32::i2c::{I2c, mode::Master};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use mpu6050::*;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use nalgebra::{Vector2, Vector3};

pub type MpuRcv = Channel<ThreadModeRawMutex, MpuData, 1>;

pub struct MpuData {
    pub acc: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub temp: f32,
    pub acc_angles: Vector2<f32>,
}

#[embassy_executor::task]
pub async fn mpu_task(
    i2c: I2c<'static, Async, Master>, 
    mut led: Output<'static>,
    rcv_mpu: & 'static MpuRcv
) {

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

    let mut acquisition_count = 0;

    // Main loop - read sensor data
    loop {
        if acquisition_count % 10 == 0 {
            led.toggle();
        }

        let acc = mpu.get_acc().unwrap();
        let gyro = mpu.get_gyro().unwrap();
        let temp = mpu.get_temp().unwrap();
        let acc_angles = mpu.get_acc_angles().unwrap();

        let mpu_data = MpuData {
            acc: Vector3::new(acc.x, acc.y, acc.z),
            gyro: Vector3::new(gyro.x, gyro.y, gyro.z),
            temp,
            acc_angles: Vector2::new(acc_angles.x, acc_angles.y),
        };

        rcv_mpu.send(mpu_data).await;

        acquisition_count += 1;
    }
}