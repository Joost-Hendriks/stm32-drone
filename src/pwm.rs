use defmt::*;
use embassy_executor;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use embassy_stm32::peripherals::TIM2;

/// Convert a throttle percentage (0–100) to a duty cycle value.
/// 0% = 1000µs pulse (idle/arm), 100% = 2000µs pulse (full throttle).
fn throttle_duty(percent: u8, max_duty: u16) -> u16 {
    let percent = percent.min(100) as u32;
    let pulse_us = 1000 + percent * 10; // 1000µs at 0%, 2000µs at 100%
    (max_duty as u32 * pulse_us / 20000) as u16
}

#[embassy_executor::task]
pub async fn pwm_task(mut pwm: SimplePwm<'static, TIM2>) {

    let max_duty = pwm.ch1().max_duty_cycle();

    // Enable all 4 channels
    pwm.ch1().enable();
    pwm.ch2().enable();
    pwm.ch3().enable();
    pwm.ch4().enable();

    // Arm all ESCs: hold 0% throttle for 3 seconds
    info!("Arming ESCs (hold 0% throttle for 3s)...");
    pwm.ch1().set_duty_cycle(throttle_duty(0, max_duty));
    pwm.ch2().set_duty_cycle(throttle_duty(0, max_duty));
    pwm.ch3().set_duty_cycle(throttle_duty(0, max_duty));
    pwm.ch4().set_duty_cycle(throttle_duty(0, max_duty));
    Timer::after(Duration::from_secs(3)).await;
    info!("ESCs armed!");

}