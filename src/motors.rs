use defmt::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use embassy_stm32::peripherals::TIM2;
use nalgebra::Vector4;

/// Per-motor normalized thrust commands in [0.0, 1.0].
/// 0.0 → 1000 µs (motor idle / armed), 1.0 → 2000 µs (full throttle).

pub struct Motors {
    pwm: SimplePwm<'static, TIM2>,
    max_duty_ch1: u32,
    max_duty_ch2: u32,
    max_duty_ch3: u32,
    max_duty_ch4: u32,
}

pub async fn initialize_motors(mut pwm: SimplePwm<'static, TIM2>) -> Motors {

    let max_duty_ch1 = pwm.ch1().max_duty_cycle();
    let max_duty_ch2 = pwm.ch2().max_duty_cycle();
    let max_duty_ch3 = pwm.ch3().max_duty_cycle();
    let max_duty_ch4 = pwm.ch4().max_duty_cycle();

    // Enable all 4 channels
    pwm.ch1().enable();
    pwm.ch2().enable();
    pwm.ch3().enable();
    pwm.ch4().enable();

    // Arm all ESCs: hold minimum throttle (1000 µs) for 3 seconds
    info!("Arming ESCs ...");
    let mut fraction = 0.0;
    pwm.ch1().set_duty_cycle(normalized_to_duty(max_duty_ch1, fraction));
    pwm.ch2().set_duty_cycle(normalized_to_duty(max_duty_ch2, fraction));
    pwm.ch3().set_duty_cycle(normalized_to_duty(max_duty_ch3, fraction));
    pwm.ch4().set_duty_cycle(normalized_to_duty(max_duty_ch4, fraction));
    Timer::after(Duration::from_secs(3)).await;
    info!("ESCs armed!");

    // Idle at 25 % throttle for 1 s before entering the control loop
    fraction = 0.3;
    pwm.ch1().set_duty_cycle(normalized_to_duty(max_duty_ch1, fraction));
    pwm.ch2().set_duty_cycle(normalized_to_duty(max_duty_ch2, fraction));
    pwm.ch3().set_duty_cycle(normalized_to_duty(max_duty_ch3, fraction));
    pwm.ch4().set_duty_cycle(normalized_to_duty(max_duty_ch4, fraction));
    Timer::after(Duration::from_secs(1)).await;

    Motors {
        pwm,
        max_duty_ch1,
        max_duty_ch2,
        max_duty_ch3,
        max_duty_ch4,
    }

}


pub async fn set_power(motors: &mut Motors, power_motors: Vector4<f32>) {

    // info!("Fraction: {:?}", power_motors[0]);
    // info!("Normalized to duty: {:?}", normalized_to_duty(motors.max_duty, power_motors[3]));
    // info!("Max duty: {:?}", motors.max_duty);

    motors.pwm.ch1().set_duty_cycle(normalized_to_duty(motors.max_duty_ch1, power_motors[0]));
    motors.pwm.ch2().set_duty_cycle(normalized_to_duty(motors.max_duty_ch2, power_motors[1]));
    motors.pwm.ch3().set_duty_cycle(normalized_to_duty(motors.max_duty_ch3, power_motors[2]));
    motors.pwm.ch4().set_duty_cycle(normalized_to_duty(motors.max_duty_ch4, power_motors[3]));
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
fn normalized_to_duty(max_duty: u32, fraction: f32) -> u32 {
    let min_count = max_duty * 1000 / 20_000;
    let max_count = max_duty * 2000 / 20_000;
    let count = min_count as f32 + ((max_count - min_count) as f32 * fraction.clamp(0.0, 1.0));
    count as u32
}
