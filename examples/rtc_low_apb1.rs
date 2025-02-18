//! Real-time clock (RTC) example with low APB1 clock frequency

#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32l0xx_hal::{
    pac,
    prelude::*,
    pwr::PWR,
    rcc,
    rtc::{ClockSource, NaiveDate, Rtc, Timelike},
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // This should put the APB1 clock at 2 times the RTC clock, if I follow the
    // code correctly. Exactly the range that is still acceptable, but requires
    // special handling in the RTC code.
    let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range0));
    let pwr = PWR::new(dp.PWR, &mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let mut led = gpioa.pa7.into_push_pull_output();
    // Signal start
    for _ in 0..10 {
        led.toggle();
        for _ in 0..1000 {
            asm::nop()
        }
    }

    // Initialize RTC
    let init = NaiveDate::from_ymd_opt(2019, 8, 9)
        .unwrap()
        .and_hms_opt(13, 37, 42)
        .unwrap();
    // If the target hardware has an external crystal, ClockSource::LSE can be used
    // instead of ClockSource::LSI for greater accuracy
    let mut rtc = Rtc::new(dp.RTC, &mut rcc, &pwr, ClockSource::LSI, Some(init)).unwrap();

    let mut last_second = 0;

    loop {
        let instant = rtc.now();
        if instant.second() != last_second {
            last_second = instant.second();

            // Given the clock settings above, this gives a good blinking going
            // if compiled in release mode.
            led.set_high().unwrap();
            for _ in 0..1 {
                asm::nop()
            }
            led.set_low().unwrap();
        }
    }
}
