#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Timer};
use mpu6050::*;
use {defmt_rtt as _, panic_probe as _};

// Bind interrupt handlers
embassy_stm32::bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting MPU6050 reader!");
    
    // Initialize STM32 peripherals
    let p = embassy_stm32::init(Default::default());

    // Initialize the onboard LED (PC13 on Black Pill)
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    info!("Initializing I2C...");
    
    // Initialize I2C1 (SCL=PB6, SDA=PB7)
    let mut i2c_config = i2c::Config::default();
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;
    
    let i2c = I2c::new(
        p.I2C1,
        p.PB6, // SCL
        p.PB7, // SDA
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        i2c_config,
    );

    // Initialize PWM on PA0 (TIM2_CH1) at 50Hz for ESC/servo control
    let ch1_pin = PwmPin::new(p.PA0, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        p.TIM2,
        Some(ch1_pin),
        None,
        None,
        None,
        hz(50),
        Default::default(),
    );
    let max_duty = pwm.ch1().max_duty_cycle();
    pwm.ch1().enable();

    // ESC expects 1000-2000µs pulses within a 20ms period (50Hz)
    // duty = pulse_us / 20000 * max_duty
    let min_throttle = max_duty as u32 * 1000 / 20000; // 1000µs = arm/idle
    let max_throttle = max_duty as u32 * 2000 / 20000; // 2000µs = full throttle

    // Arm ESC: hold minimum throttle for 3 seconds
    info!("Arming ESC (hold min throttle for 3s)...");
    pwm.ch1().set_duty_cycle(min_throttle as u16);
    Timer::after(Duration::from_secs(3)).await;
    info!("ESC armed! max_duty: {}, min_throttle: {}, max_throttle: {}", max_duty, min_throttle, max_throttle);

    info!("I2C initialized, connecting to MPU6050...");

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

    // Example: 25% throttle
    let throttle = min_throttle + (max_throttle - min_throttle) * 25 / 100;
    pwm.ch1().set_duty_cycle(throttle as u16);


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