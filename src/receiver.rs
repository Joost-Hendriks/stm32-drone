use embassy_stm32::exti::ExtiInput;
use embassy_time::Instant;

/// Last-received PWM channel values (1000–2000 µs range, default 1500).
///
/// Channel layout:
///   [0] CH1 – Yaw
///   [1] CH2 – Pitch
///   [2] CH3 – Thrust
///   [3] CH4 – Roll
///   [4] CH5 – Var 1
///   [5] CH6 – Var 2


/// Measures a single PWM pulse and returns its width in microseconds.
pub async fn pwm_receiver_channel(pin: &mut ExtiInput<'static>) -> u64 {
    pin.wait_for_rising_edge().await;
    let t_rise = Instant::now();

    pin.wait_for_falling_edge().await;
    let pulse_us = t_rise.elapsed().as_micros();

    // info!("CH{}: {}µs", idx, pulse_us);
    pulse_us
}
