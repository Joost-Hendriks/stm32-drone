use defmt::*;
use embassy_stm32::i2c::{I2c, mode::Master};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use mpu6050::*;
use nalgebra::Vector3;

pub struct MpuData {
    pub acc: Vector3<f32>,
    pub gyro: Vector3<f32>,
}

pub async fn initialize_mpu(i2c: I2c<'static, Async, Master>) -> Mpu6050<I2c<'static, Async, Master>> {

    info!("Connecting to MPU6050...");

    // Initialize MPU6050 (default address 0x68)
    let mut mpu = Mpu6050::new(i2c);
    
    // Wait for sensor to stabilize
    Timer::after(Duration::from_millis(100)).await;
    
    // Initialize the sensor
    let mut delay = embassy_time::Delay;
    match mpu.init(&mut delay) {
        Ok(_) => info!("MPU6050 initialized successfully!"),
        Err(_) => error!("Failed to initialize MPU6050!"),
    }
    mpu
}

pub async fn read_mpu(mpu: &mut Mpu6050<I2c<'static, Async, Master>>) -> MpuData {

    let acc = mpu.get_acc().unwrap();
    let gyro = mpu.get_gyro().unwrap();

    MpuData {
        acc: Vector3::new(acc.x, acc.y, acc.z),
        gyro: Vector3::new(gyro.x, gyro.y, gyro.z),
    }
}