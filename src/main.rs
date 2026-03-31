#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::peripherals;
use static_cell::StaticCell;
use mpu6050::*;
use {defmt_rtt as _, panic_probe as _};

mod mpu;
mod motors;
mod receiver;
mod controller;
mod pid;

use crate::mpu::initialize_mpu;
use crate::motors::initialize_motors;
use crate::controller::control_loop;

// Bind interrupt handlers
embassy_stm32::bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    EXTI4   => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
});

static MOTORS: StaticCell<motors::Motors> = StaticCell::new();
static MPU: StaticCell<Mpu6050<I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::mode::Master>>> = StaticCell::new();

static CH1: StaticCell<ExtiInput> = StaticCell::new();
static CH2: StaticCell<ExtiInput> = StaticCell::new();
static CH3: StaticCell<ExtiInput> = StaticCell::new();
static CH4: StaticCell<ExtiInput> = StaticCell::new();
static CH5: StaticCell<ExtiInput> = StaticCell::new();
static CH6: StaticCell<ExtiInput> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting STM32");

    // Initialize STM32 peripherals
    let p = embassy_stm32::init(Default::default());

    // Initialize the onboard LED (PC13 on Black Pill)
    let led = Output::new(p.PC13, Level::High, Speed::Low);

    info!("Initializing I2C...");

    // Initialize I2C1 (SCL=PB6, SDA=PB7)
    let mut i2c_config = i2c::Config::default();
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;
    i2c_config.frequency = hz(200_000);

    let i2c = I2c::new(
        p.I2C1,
        p.PB6, // SCL
        p.PB7, // SDA
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        i2c_config,
    );

    info!("I2C initialized");

    let mpu = initialize_mpu(i2c).await;
    let mpu = MPU.init(mpu);

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

    info!("PWM motors initialized");

    let motors = MOTORS.init(initialize_motors(pwm).await);


    let ch1 = CH1.init(ExtiInput::new(p.PA4, p.EXTI4, Pull::None, Irqs));
    let ch2 = CH2.init(ExtiInput::new(p.PA5, p.EXTI5, Pull::None, Irqs));
    let ch3 = CH3.init(ExtiInput::new(p.PA6, p.EXTI6, Pull::None, Irqs));
    let ch4 = CH4.init(ExtiInput::new(p.PA7, p.EXTI7, Pull::None, Irqs));
    let ch5 = CH5.init(ExtiInput::new(p.PA8, p.EXTI8, Pull::None, Irqs));
    let ch6 = CH6.init(ExtiInput::new(p.PA9, p.EXTI9, Pull::None, Irqs));

    let channels: [&'static mut ExtiInput<'static>; 6] = [ch1, ch2, ch3, ch4, ch5, ch6];


    info!("PWM receiver initialized");

    info!("Starting tasks");
    control_loop(mpu, motors, led, channels).await;

    // _ = spawner.spawn(control_loop(mpu, motors, led, channels)).map_err(|_| error!("Error running control loop"));
    // _ = spawner.spawn(pwm_receiver_channel(ch1, 0)).map_err(|_| error!("Error running receiver CH1"));
    // _ = spawner.spawn(pwm_receiver_channel(ch2, 1)).map_err(|_| error!("Error running receiver CH2"));
    // _ = spawner.spawn(pwm_receiver_channel(ch3, 2)).map_err(|_| error!("Error running receiver CH3"));
    // _ = spawner.spawn(pwm_receiver_channel(ch4, 3)).map_err(|_| error!("Error running receiver CH4"));
    // _ = spawner.spawn(pwm_receiver_channel(ch5, 4)).map_err(|_| error!("Error running receiver CH5"));
    // _ = spawner.spawn(pwm_receiver_channel(ch6, 5)).map_err(|_| error!("Error running receiver CH6"));

}
