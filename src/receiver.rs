use defmt::*;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Instant;

/// Last-received PWM channel values (1000–2000 µs range, default 1500).
///
/// Channel layout:
///   [0] CH1 – Aileron (roll)
///   [1] CH2 – Elevator (pitch)
///   [2] CH3 – Throttle
///   [3] CH4 – Rudder (yaw)
///   [4] CH5 – SWA (2-pos switch)
///   [5] CH6 – SWB (2-pos switch)

#[embassy_executor::task(pool_size = 6)]
pub async fn pwm_receiver_channel(mut pin: ExtiInput<'static>, idx: usize) -> ! {
    info!("PWM receiver channel {} started", idx);

    loop {
        pin.wait_for_rising_edge().await;
        let t_rise = Instant::now();

        pin.wait_for_falling_edge().await;
        let pulse_us = t_rise.elapsed().as_micros();

        info!("CH{}: {}µs", idx, pulse_us);
    }
}
