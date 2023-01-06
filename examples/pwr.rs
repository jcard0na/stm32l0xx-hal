#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32l0xx_hal::{
    exti::{ConfigurableLine, Exti, TriggerEdge},
    gpio::{Output, Pin, PushPull},
    pac,
    prelude::*,
    pwr::{self, PWR},
    rcc::{self, Rcc},
    rtc::{self, ClockSource, Rtc},
};


#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut scb = cp.SCB;
    let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range0));
    let mut exti = Exti::new(dp.EXTI);
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut rled = gpioa.pa5.into_push_pull_output().downgrade();
    let mut gled = gpiob.pb4.into_push_pull_output().downgrade();

    // Initialize RTC
    let mut rtc = Rtc::new(dp.RTC, &mut rcc, &pwr, ClockSource::LSI, None).unwrap();

    // Enable interrupts
    let exti_line = ConfigurableLine::RtcWakeup;
    rtc.enable_interrupts(rtc::Interrupts {
        wakeup_timer: true,
        ..rtc::Interrupts::default()
    });
    exti.listen_configurable(exti_line, TriggerEdge::Rising);

    let mut timer = rtc.wakeup_timer();

    // Blink twice to signal the start of the program
    blink(&mut rled);
    blink(&mut gled);

    // 20 seconds of stop mode
    timer.start(20u32);
    prepare_to_stop(&mut rcc);
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
    timer.wait().unwrap(); // returns immediately; we just got the interrupt

    // signal that we are entering standby mode
    blink(&mut rled);
    blink(&mut gled);
    blink(&mut rled);

    // 20 more seconds of standby mode
    cortex_m::peripheral::NVIC::unpend(pac::Interrupt::RTC);
    exti.wait_for_irq(exti_line, pwr.standby_mode(&mut scb));

    // The microcontroller resets after leaving standby mode. We should never
    // reach this point.
    loop {
        blink(&mut rled);
    }
}

fn prepare_to_stop(rcc: &mut Rcc) {
    // We are about to stop, and then reset, so it is ok to break all the rules
    // and work directly on peripherals that might have been used before.
    unsafe {
        let p = pac::Peripherals::steal();

        let spi1 = p.SPI1;
        let spi2 = p.SPI2;

        // wait for all spi activity to end
        while spi1.sr.read().bsy().bit_is_set() || spi2.sr.read().bsy().bit_is_set() {}

        let gpiob = p.GPIOB.split(rcc);
        let gpioa = p.GPIOA.split(rcc);

        let _ = gpiob.pb0.into_analog();
        let _ = gpiob.pb1.into_analog();
        let _ = gpiob.pb2.into_analog();
        let _ = gpiob.pb3.into_analog();
        // led, leave alone
        // let _ = gpiob.pb4.into_analog();
        let _ = gpiob.pb5.into_analog();
        let _ = gpiob.pb6.into_analog();
        let _ = gpiob.pb7.into_analog();
        let _ = gpiob.pb8.into_analog();
        let _ = gpiob.pb9.into_analog();
        let _ = gpiob.pb10.into_analog();
        let _ = gpiob.pb11.into_analog();
        let _ = gpiob.pb12.into_analog();
        let _ = gpiob.pb13.into_analog();
        let _ = gpiob.pb14.into_analog();
        let _ = gpiob.pb15.into_analog();

        let _ = gpioa.pa0.into_analog();
        let _ = gpioa.pa1.into_analog();
        let _ = gpioa.pa2.into_analog();
        let _ = gpioa.pa3.into_analog();
        let _ = gpioa.pa4.into_analog();
        // led, leave alone
        // let _ = gpioa.pa5.into_analog();
        let _ = gpioa.pa6.into_analog();
        let _ = gpioa.pa7.into_analog();
        let _ = gpioa.pa8.into_analog();
        let _ = gpioa.pa9.into_analog();
        let _ = gpioa.pa10.into_analog();
        let _ = gpioa.pa11.into_analog();
        let _ = gpioa.pa12.into_analog();
        // debug pins, leave alone
        // let _ = gpioa.pa13.into_analog();
        // let _ = gpioa.pa14.into_analog();
        let _ = gpioa.pa15.into_analog();

        // Disable all gpio clocks
        rcc.iopenr.modify(|_, w| w.iopaen().disabled());
        rcc.iopenr.modify(|_, w| w.iopben().disabled());
        rcc.iopenr.modify(|_, w| w.iopcen().disabled());
        rcc.iopenr.modify(|_, w| w.iopden().disabled());
        rcc.iopenr.modify(|_, w| w.iopeen().disabled());
        rcc.iopenr.modify(|_, w| w.iophen().disabled());

        // Disable adc clock
        rcc.apb2enr.modify(|_, w| w.adcen().disabled());
    }
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
