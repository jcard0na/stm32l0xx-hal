#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, pac::SCB, prelude::*, rcc::Config};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPI0A and GPIOB peripherals. This also enables the clock for
    // GPIOA and GPIOB in the RCC register.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Configure PB12 as input.
    let button = gpiob.pb12.into_pull_up_input();

    // Configure PA7 as output.
    let mut led = gpioa.pa7.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(rcc.clocks);

    for _ in 0..40 {
        led.toggle().unwrap();
        delay.delay(30.milliseconds());
    }

    loop {
        match button.is_high() {
            Ok(true) => SCB::sys_reset(),
            Ok(false) => (),
            _ => unreachable!(),
        };
        delay.delay(300.milliseconds());
        led.toggle().unwrap();
    }
}
