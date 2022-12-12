#![no_main]
#![no_std]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use cortex_m::asm;
use cortex_m_rt::entry;
// use cortex_m_semihosting::hprintln;
use stm32l0xx_hal::{
    exti::{ConfigurableLine, Exti, TriggerEdge},
    gpio::{Output, Pin, PushPull},
    pac,
    prelude::*,
    pwr::{self, PWR},
    rcc,
    rtc::{self, ClockSource, Rtc},
    spi::MODE_0,
};


use spi_memory::series25::Flash;
use lis3dh_spi::{
    ctrl_reg_1_value::{CtrlReg1Value, LPEn, XEn, YEn, ZEn, ODR},
};

#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut scb = cp.SCB;
    let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range6));
    let mut exti = Exti::new(dp.EXTI);
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let _ = gpioa.pa0.into_analog();
    let _ = gpioa.pa1.into_analog();
    let _ = gpioa.pa2.into_analog();
    let mut supercap_read_en = gpioa.pa4.into_push_pull_output().downgrade();
    let _ = gpioa.pa5.into_analog();
    let _ = gpioa.pa6.into_analog();
    let _ = gpioa.pa7.into_analog();
    let mut led = gpioa.pa8.into_push_pull_output().downgrade();
    let _ = gpioa.pa9.into_analog();
    let _ = gpioa.pa10.into_analog();
    let _ = gpioa.pa12.into_analog();
    // let _ = gpioa.pa13.into_analog();
    // let _ = gpioa.pa14.into_analog();
    let _ = gpioa.pa15.into_analog();

    let _ = gpiob.pb0.into_analog();
    let _ = gpiob.pb1.into_analog();
    let _ = gpiob.pb2.into_analog();
    let sck = gpiob.pb3;
    let miso = gpiob.pb4;
    let mosi = gpiob.pb5;
    let mut cs_flash = gpiob.pb6.into_push_pull_output();
    let _ = gpiob.pb7.into_analog();
    let mut cs_accel = gpiob.pb8.into_push_pull_output();
    let _ = gpiob.pb9.into_analog();
    let _ = gpiob.pb10.into_analog();
    let _ = gpiob.pb11.into_analog();
    let _ = gpiob.pb12.into_analog();
    let _ = gpiob.pb13.into_analog();
    let _ = gpiob.pb14.into_analog();
    let _ = gpiob.pb15.into_analog();

    supercap_read_en.set_low().unwrap();

    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), MODE_0, 2_000_000.Hz(), &mut rcc);

    let mut accelerometer = lis3dh_spi::Lis3dh::default();
    let mut accel_cfg1 = CtrlReg1Value::default();
    accel_cfg1.set_x_en(XEn::XAxisDisabled);
    accel_cfg1.set_y_en(YEn::YAxisDisabled);
    accel_cfg1.set_z_en(ZEn::ZAxisDisabled);
    accel_cfg1.set_output_data_rate(ODR::PowerDownMode);
    accel_cfg1.set_l_p_en(LPEn::LowPowerEnabled);
    accelerometer.set_ctrl_reg1_setting(accel_cfg1);
    accelerometer.write_all_settings(&mut cs_accel, &mut spi).ok();

    if !accelerometer
        .check_if_settings_are_written_correctly(&mut cs_accel, &mut spi)
        .unwrap()
    {
        // NOTE: When the accelerometer is configured before epd.set_lut() command
        // is issued, we observe the accelerometer to transition into power down
        // state (ODR all zeros).  If we go to sleep in that state, the
        // we cannot respond to accel events.
        // TODO: This is probably fixed in version 1.6, so we might be able to move
        // up the accelerometer configuration.
        blink(&mut led);
        blink(&mut led);
        blink(&mut led);
        blink(&mut led);
        blink(&mut led);
        blink(&mut led);
        blink(&mut led);
    }

    // // let mut spi2 = p
    // //     .SPI2
    // //     .spi((sck2, miso2, mosi2), MODE_3, 2_000_000.Hz(), &mut rcc);

    cs_flash.set_low().unwrap();
    // hprintln!("one");
    blink(&mut led);
    cs_flash.set_high().unwrap();
    let flash = Flash::init(spi, cs_flash);
    blink(&mut led);
    let mut flash = flash.unwrap();
    blink(&mut led);
    flash.sleep().unwrap();
    blink(&mut led);

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

    // // 5 seconds of regular run mode
    // timer.start(5u32);
    // while let Err(nb::Error::WouldBlock) = timer.wait() {}
    // Exti::unpend(exti_line);
    // NVIC::unpend(pac::Interrupt::RTC);

    // blink(&mut led);

    // // 5 seconds of low-power run mode
    // pwr.enter_low_power_run_mode(rcc.clocks);
    // while let Err(nb::Error::WouldBlock) = timer.wait() {}
    // pwr.exit_low_power_run_mode();
    // Exti::unpend(exti_line);
    // NVIC::unpend(pac::Interrupt::RTC);

    // blink(&mut led);

    // // 5 seconds of sleep mode
    // exti.wait_for_irq(exti_line, pwr.sleep_mode(&mut scb));
    // timer.wait().unwrap(); // returns immediately; we just got the interrupt

    // blink(&mut led);

    // // 5 seconds of low-power sleep mode
    // exti.wait_for_irq(exti_line, pwr.low_power_sleep_mode(&mut scb, &mut rcc));
    // timer.wait().unwrap(); // returns immediately; we just got the interrupt

    // blink(&mut led);

    // Disable all gpio clocks
    rcc.iopenr.modify(|_, w| w.iopaen().disabled());
    rcc.iopenr.modify(|_, w| w.iopben().disabled());
    rcc.iopenr.modify(|_, w| w.iopcen().disabled());
    rcc.iopenr.modify(|_, w| w.iopden().disabled());
    rcc.iopenr.modify(|_, w| w.iopeen().disabled());
    rcc.iopenr.modify(|_, w| w.iophen().disabled());

    // 20 seconds of stop mode
    timer.start(20u32);
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

    // enable clocks we need for LED and spi
    rcc.iopenr.modify(|_, w| w.iopaen().enabled());
    rcc.iopenr.modify(|_, w| w.iopben().enabled());

    // blink to indicate we exited stop mode
    blink(&mut led);
    timer.wait().unwrap(); // returns immediately; we just got the interrupt

    flash.wakeup().unwrap();

    // signal that we are entering standby mode
    blink(&mut led);

    // 5 seconds of standby mode
    cortex_m::peripheral::NVIC::unpend(pac::Interrupt::RTC);
    exti.wait_for_irq(exti_line, pwr.standby_mode(&mut scb));

    // The microcontroller resets after leaving standby mode. We should never
    // reach this point.
    loop {
        blink(&mut led);
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
    for _ in 0..100_000 {
        asm::nop()
    }
}
