use defmt::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use embassy_stm32::peripherals::TIM2;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use nalgebra::Vector4;

/// Per-motor normalized thrust commands in [0.0, 1.0].
/// 0.0 → 1000 µs (motor idle / armed), 1.0 → 2000 µs (full throttle).
pub type MotorsCmd = Channel<ThreadModeRawMutex, Vector4<f32>, 1>;

#[embassy_executor::task]
pub async fn motors_task(mut pwm: SimplePwm<'static, TIM2>, cmd_channel: & 'static MotorsCmd) -> ! {

    let max_duty = pwm.ch1().max_duty_cycle();

    // Enable all 4 channels
    pwm.ch1().enable();
    pwm.ch2().enable();
    pwm.ch3().enable();
    pwm.ch4().enable();

    // Arm all ESCs: hold minimum throttle (1000 µs) for 3 seconds
    info!("Arming ESCs ...");
    let arm = normalized_to_duty(max_duty, 0.0);
    pwm.ch1().set_duty_cycle(arm);
    pwm.ch2().set_duty_cycle(arm);
    pwm.ch3().set_duty_cycle(arm);
    pwm.ch4().set_duty_cycle(arm);
    Timer::after(Duration::from_secs(3)).await;
    info!("ESCs armed!");

    // Idle at 25 % throttle for 1 s before entering the control loop
    let idle = normalized_to_duty(max_duty, 0.25);
    pwm.ch1().set_duty_cycle(idle);
    pwm.ch2().set_duty_cycle(idle);
    pwm.ch3().set_duty_cycle(idle);
    pwm.ch4().set_duty_cycle(idle);
    Timer::after(Duration::from_secs(1)).await;

    loop {
        let cmds = cmd_channel.receive().await;

        pwm.ch1().set_duty_cycle(normalized_to_duty(max_duty, cmds[0]));
        pwm.ch2().set_duty_cycle(normalized_to_duty(max_duty, cmds[1]));
        pwm.ch3().set_duty_cycle(normalized_to_duty(max_duty, cmds[2]));
        pwm.ch4().set_duty_cycle(normalized_to_duty(max_duty, cmds[3]));
    }
}

/// Convert a normalized thrust command [0.0, 1.0] to a PWM duty-cycle count.
///
/// ESCs expect a 1000–2000 µs pulse within a 20 ms (50 Hz) period:
///   duty_count = pulse_us / 20_000 * max_duty
///
/// So:
///   min_count = max_duty * 1000 / 20_000  (1000 µs → idle/armed)
///   max_count = max_duty * 2000 / 20_000  (2000 µs → full throttle)
///   duty_count = min_count + fraction * (max_count - min_count)
fn normalized_to_duty(max_duty: u16, fraction: f32) -> u16 {
    let min_count = max_duty as u32 * 1000 / 20_000;
    let max_count = max_duty as u32 * 2000 / 20_000;
    let count = min_count + ((max_count - min_count) as f32 * fraction.clamp(0.0, 1.0)) as u32;
    count as u16
}
