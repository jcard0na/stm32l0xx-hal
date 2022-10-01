//! Real-time clock (RTC) example
//!
//! This example initializes the RTC with a specific time and prints this time
//! to the USART. You should see this time counting up, even if you reset the
//! device, or hold it in reset for a while.
//!
//! If you disconnect the device from power, the RTC should reset to the initial
//! time programmed here after you connect it again. If you press the button,
//! that should rapidly change the seconds while you hold it down.

#![no_main]
#![no_std]

extern crate panic_semihosting;
use cortex_m_semihosting::hprintln;
use cortex_m_rt::entry;

use stm32l0xx_hal::{
    pac,
    prelude::*,
    pwr::PWR,
    rcc,
    rtc::{ClockSource, Rtc, Timelike, Datelike},
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let pwr = PWR::new(dp.PWR, &mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let _button = gpiob.pb12.into_floating_input();

    // If the target hardware has an external crystal, ClockSource::LSE can be used
    // instead of ClockSource::LSI for greater accuracy
    let mut rtc = Rtc::new(dp.RTC, &mut rcc, &pwr, ClockSource::LSI, None).unwrap();

    loop {
        let instant = rtc.now();

        hprintln!(
            "{:02}-{:02}-{:02} {:02}:{:02}:{:02}",
            instant.year(),
            instant.month(),
            instant.day(),
            instant.hour(),
            instant.minute(),
            instant.second(),
        );
    }
}
