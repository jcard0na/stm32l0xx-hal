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
    spi::{MODE_0, MODE_3},
};

use lis3dh_spi::ctrl_reg_1_value::{CtrlReg1Value, XEn, YEn, ZEn, ODR};

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
    let _supercap_read_en = gpioa.pa4.into_pull_down_input();
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
    let cs_flash = gpiob.pb6.into_push_pull_output();
    let _ = gpiob.pb7.into_analog();
    let mut cs_accel = gpiob.pb8.into_push_pull_output();
    let _accel_off = gpiob.pb9.into_pull_down_input();
    let _ = gpiob.pb10.into_analog();
    let _ = gpiob.pb11.into_analog();
    let _ = gpiob.pb12.into_analog();
    let sck2 = gpiob.pb13;
    let miso2 = gpiob.pb14;
    let mosi2 = gpiob.pb15;

    // wait for accel to boot
    delay();

    let spi = dp
        .SPI1
        .spi((sck, miso, mosi), MODE_0, 2_000_000.Hz(), &mut rcc);

    let mut spi2 = dp
        .SPI2
        .spi((sck2, miso2, mosi2), MODE_3, 2_000_000.Hz(), &mut rcc);

    let mut accelerometer = lis3dh_spi::Lis3dh::default();
    let mut accel_cfg1 = CtrlReg1Value::default();
    // This configuration draws 73 uA, confirmed experimentally
    // accel_cfg1.set_x_en(XEn::XAxisEnabled);
    // accel_cfg1.set_y_en(YEn::YAxisEnabled);
    // accel_cfg1.set_z_en(ZEn::ZAxisEnabled);
    // accel_cfg1.set_output_data_rate(ODR::Hz400);
    // This configuration draws 1 uA, confirmed experimentally
    accel_cfg1.set_x_en(XEn::XAxisDisabled);
    accel_cfg1.set_y_en(YEn::YAxisDisabled);
    accel_cfg1.set_z_en(ZEn::ZAxisDisabled);
    accel_cfg1.set_output_data_rate(ODR::PowerDownMode);
    accelerometer.set_ctrl_reg1_setting(accel_cfg1);
    accelerometer
        .write_all_settings(&mut cs_accel, &mut spi2)
        .ok();
    blink(&mut led);
    if !accelerometer
        .check_if_settings_are_written_correctly(&mut cs_accel, &mut spi2)
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

    // release spi2
    cs_accel.into_pull_up_input();
    let (spi2, pins) = spi2.free();
    spi2.cr1.modify(|_, w| w.spe().clear_bit());
    let sck2 = pins.0;
    // Mode 3, idle high
    sck2.into_pull_up_input();
    let miso2 = pins.1;
    miso2.into_analog();
    let mosi2 = pins.2;
    mosi2.into_pull_down_input();

    // // let mut spi2 = p
    // //     .SPI2
    // //     .spi((sck2, miso2, mosi2), MODE_3, 2_000_000.Hz(), &mut rcc);

    // let flash = Flash::init(spi, cs_flash);
    blink(&mut led);
    // let mut flash = flash.unwrap();
    blink(&mut led);
    // flash.sleep().unwrap();
    blink(&mut led);

    // release spi1
    // let (spi, cs_flash) = flash.release();
    cs_flash.into_pull_up_input();
    let (spi1, pins) = spi.free();
    spi1.cr1.modify(|_, w| w.spe().clear_bit());
    let sck = pins.0;
    // Mode 0, idle low
    sck.into_pull_down_input();
    let miso = pins.1;
    miso.into_analog();
    let mosi = pins.2;
    mosi.into_pull_down_input();

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

    // Disable all gpio clocks
    rcc.iopenr.modify(|_, w| w.iopaen().disabled());
    rcc.iopenr.modify(|_, w| w.iopben().disabled());
    rcc.iopenr.modify(|_, w| w.iopcen().disabled());
    rcc.iopenr.modify(|_, w| w.iopden().disabled());
    rcc.iopenr.modify(|_, w| w.iopeen().disabled());
    rcc.iopenr.modify(|_, w| w.iophen().disabled());

    // 20 seconds of stop mode
    timer.start(20u32);

    loop {
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
        // next call will return immediately, as it was the timer interrupt that
        // woke us up
        timer.wait().unwrap();

        // give signs of life and go back to stop
        rcc.iopenr.modify(|_, w| w.iopaen().enabled());
        blink(&mut led);
        rcc.iopenr.modify(|_, w| w.iopaen().disabled());
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
