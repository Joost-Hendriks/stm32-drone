#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use {defmt_rtt as _, panic_probe as _};

mod mpu;
mod pwm;

use crate::pwm::pwm_task;
use crate::mpu::mpu_task;

// Bind interrupt handlers
embassy_stm32::bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting MPU6050 reader!");
    
    // Initialize STM32 peripherals
    let p = embassy_stm32::init(Default::default());

    // Initialize the onboard LED (PC13 on Black Pill)
    let led = Output::new(p.PC13, Level::High, Speed::Low);

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

    // Initialize PWM on TIM2 CH1-CH4 at 50Hz for ESC/servo control
    // CH1=PA0, CH2=PA1, CH3=PA2, CH4=PA3
    let ch1_pin = PwmPin::new(p.PA0, OutputType::PushPull);
    let ch2_pin = PwmPin::new(p.PA1, OutputType::PushPull);
    let ch3_pin = PwmPin::new(p.PA2, OutputType::PushPull);
    let ch4_pin = PwmPin::new(p.PA3, OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM2,
        Some(ch1_pin),
        Some(ch2_pin),
        Some(ch3_pin),
        Some(ch4_pin),
        hz(50),
        Default::default(),
    );

    info!("I2C initialized");

    _ = spawner.spawn(pwm_task(pwm)).map_err(|_| error!("Error running pwm task"));
    _ = spawner.spawn(mpu_task(i2c, led)).map_err(|_| error!("Error running pwm task"));

}