use defmt::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use embassy_stm32::peripherals::TIM2;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use nalgebra::Vector4;

pub type MotorsCmd = Channel<ThreadModeRawMutex, Vector4<u16>, 1>;

#[embassy_executor::task]
pub async fn motors_task(mut pwm: SimplePwm<'static, TIM2>, cmd_channel: & 'static MotorsCmd) -> ! {

    let max_duty = pwm.ch1().max_duty_cycle();

    // Enable all 4 channels
    pwm.ch1().enable();
    pwm.ch2().enable();
    pwm.ch3().enable();
    pwm.ch4().enable();

    // Arm all ESCs: hold minimum throttle for 3 seconds
    info!("Arming ESCs ...");
    pwm.ch1().set_duty_cycle(throttle(max_duty, 1));
    pwm.ch2().set_duty_cycle(throttle(max_duty, 1));
    pwm.ch3().set_duty_cycle(throttle(max_duty, 1));
    pwm.ch4().set_duty_cycle(throttle(max_duty, 1));
    Timer::after(Duration::from_secs(3)).await;
    info!("ESCs armed!");

    pwm.ch1().set_duty_cycle(throttle(max_duty, 25));
    pwm.ch2().set_duty_cycle(throttle(max_duty, 25));
    pwm.ch3().set_duty_cycle(throttle(max_duty, 25));
    pwm.ch4().set_duty_cycle(throttle(max_duty, 25));
    Timer::after(Duration::from_secs(1)).await;

    loop {
        let duty_cyles = cmd_channel.receive().await;

        pwm.ch1().set_duty_cycle(duty_cyles[0].clamp(1000, 2000));
        pwm.ch2().set_duty_cycle(duty_cyles[1].clamp(1000, 2000));
        pwm.ch3().set_duty_cycle(duty_cyles[2].clamp(1000, 2000));
        pwm.ch4().set_duty_cycle(duty_cyles[3].clamp(1000, 2000));
    }
}

fn throttle(max_duty: u16, percentage: u32) -> u16 {
    // ESC expects 1000-2000µs pulses within a 20ms period (50Hz)
    // duty = pulse_us / 20000 * max_duty
    let min_throttle = max_duty as u32 * 1000 / 20000; // 1000µs = arm/idle
    let max_throttle = max_duty as u32 * 2000 / 20000; // 2000µs = full throttle

    let throttle = min_throttle + (max_throttle - min_throttle) * percentage / 100;
    throttle as u16
}