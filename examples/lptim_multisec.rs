//! Low-Power Timer wakeup.

#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32l0xx_hal::{
    exti::{DirectLine, Exti},
    gpio::{Output, Pin, PushPull},
    lptim::{self, ClockSrc, LpTimer},
    pac,
    prelude::*,
    pwr::{self, PWR},
    rcc,
};
use embedded_time::duration::Microseconds;

#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut scb = cp.SCB;
    let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range0));
    let mut exti = Exti::new(dp.EXTI);
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let mut ledg = gpioa.pa7.into_push_pull_output().downgrade();
    let mut ledb = gpioa.pa8.into_push_pull_output().downgrade();

    let mut lptim = LpTimer::init_oneshot(dp.LPTIM, &mut pwr, &mut rcc, ClockSrc::Lsi);

    let exti_line = DirectLine::Lptim1;

    lptim.enable_interrupts(lptim::Interrupts {
        autoreload_match: true,
        ..lptim::Interrupts::default()
    });
    exti.listen_direct(exti_line);

    // Blink twice to signal the start of the program
    blink(&mut ledg);
    blink(&mut ledg);

    lptim.start(Microseconds(10_000_000u32));

    // 10 seconds of stop mode
    exti.wait_for_irq(
        exti_line,
        pwr.stop_mode(
            &mut scb,
            &mut rcc,
            pwr::StopModeConfig {
                ultra_low_power: true,
            },
        ),
    );
    lptim.wait().unwrap(); // returns immediately; we just got the interrupt

    blink(&mut ledb);
    blink(&mut ledb);

    pac::SCB::sys_reset();
}

fn blink(led: &mut Pin<Output<PushPull>>) {
    led.set_high().unwrap();
    delay();
    led.set_low().unwrap();
    delay();
}

fn delay() {
    // We can't use `Delay`, as that requires a frequency of at least one MHz.
    // Given our clock selection, the following loop should give us a nice delay
    // when compiled in release mode.
    for _ in 0..1_000 {
        asm::nop()
    }
}
