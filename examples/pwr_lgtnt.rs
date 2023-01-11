#![no_main]
#![no_std]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use cortex_m::asm;
use cortex_m_rt::entry;
// use cortex_m_semihosting::hprintln;
use stm32l0xx_hal::{
    adc::Adc,
    delay::Delay,
    exti::{ConfigurableLine, Exti, TriggerEdge},
    gpio::{Output, Pin, PushPull},
    pac,
    prelude::*,
    pwr::{self, PWR},
    rcc,
    rtc::{self, ClockSource, Rtc},
    spi::{MODE_0, MODE_3},
};
use spi_memory::series25::Flash;

// For GDE015OC1 use:
// use epd_waveshare::{epd1in54::*, prelude::*};
// For GDEH0154D67 use:
use epd_waveshare::{epd1in54_v2::*, prelude::*};

use lis3dh_spi::ctrl_reg_1_value::{CtrlReg1Value, XEn, YEn, ZEn, ODR, LPEn};

#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // If using MSI clock instead of HSI16, uncomment block below
    // Enable ADC clock.  This is only needed when using PCLK as ADC clock
    // The alternative is to use HSI16 clock, but system clock is MSI
    // See https://github.com/stm32-rs/stm32l0xx-hal/issues/135
    dp.RCC.apb2enr.modify(|_, w| w.adcen().set_bit());

    let mut scb = cp.SCB;
    let mut rcc = dp.RCC.freeze(rcc::Config::msi(rcc::MSIRange::Range6));
    let mut exti = Exti::new(dp.EXTI);
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let mut delay = Delay::new(cp.SYST, rcc.clocks);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut solar_in = gpioa.pa0.into_analog();
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

    let rst = gpiob.pb0.into_push_pull_output();
    let dc = gpiob.pb1.into_push_pull_output();
    let cs_epd = gpiob.pb2.into_push_pull_output();
    let sck = gpiob.pb3;
    let miso = gpiob.pb4;
    let mosi = gpiob.pb5;
    let cs_flash = gpiob.pb6.into_push_pull_output();
    let busy_in = gpiob.pb7.into_floating_input();
    let mut cs_accel = gpiob.pb8.into_push_pull_output();
    let accel_off = gpiob.pb9.into_pull_down_input();
    let _ = gpiob.pb10.into_analog();
    let _ = gpiob.pb11.into_analog();
    let _ = gpiob.pb12.into_analog();
    let sck2 = gpiob.pb13;
    let miso2 = gpiob.pb14;
    let mosi2 = gpiob.pb15;

    // Enable ACD in low frequency mode, needed because we run out of the MSI clock at 2MHz and
    // datasheet says this need to be enabled below 3.5 MHz
    dp.ADC.ccr.modify(|_, w| w.lfmen().set_bit());

    // Enable internal reference voltage
    dp.ADC.ccr.modify(|_, w| w.vrefen().set_bit());

    // Pick PCKL clock for ADC if using MSI
    dp.ADC.cfgr2.modify(|_, w| w.ckmode().pclk());

    // Calibrate ADC:  without this step, we get a 4% error in
    // the calculation of VDD
    dp.ADC.cr.modify(|_, w| w.adcal().set_bit());
    while dp.ADC.isr.read().eocal().is_complete() {
        delay.delay_us(1u32);
    }

    let mut adc = Adc::new(dp.ADC, &mut rcc);
    let dummy_read: u16 = adc.read(&mut solar_in).unwrap();

    // wait for accel to boot
    delay.delay_ms(1u32);

    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), MODE_0, 2_000_000.Hz(), &mut rcc);

    let mut spi2 = dp
        .SPI2
        .spi((sck2, miso2, mosi2), MODE_3, 2_000_000.Hz(), &mut rcc);

    let mut accelerometer = lis3dh_spi::Lis3dh::default();
    let mut accel_cfg1 = CtrlReg1Value::default();
    // This disables the accelerometer.  Useful to measure the absolute
    // lowest current draw.
    // accel_cfg1.set_x_en(XEn::XAxisDisabled);
    // accel_cfg1.set_y_en(YEn::YAxisDisabled);
    // accel_cfg1.set_z_en(ZEn::ZAxisDisabled);
    // accel_cfg1.set_output_data_rate(ODR::PowerDownMode);
    // Measured that this configuration draws 10 uA over previous (disabled)
    // configuration:
    accel_cfg1.set_x_en(XEn::XAxisEnabled);
    accel_cfg1.set_y_en(YEn::YAxisEnabled);
    accel_cfg1.set_z_en(ZEn::ZAxisEnabled);
    accel_cfg1.set_output_data_rate(ODR::Hz100);
    accel_cfg1.set_l_p_en(LPEn::LowPowerEnabled);
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
    let (_, pins) = spi2.free();
    let sck2 = pins.0;
    // Mode 3, idle high
    sck2.into_pull_up_input();
    let miso2 = pins.1;
    miso2.into_analog();
    let mosi2 = pins.2;
    mosi2.into_pull_down_input();

    // turn off accelerator
    accel_off.into_analog();

    // put display to sleep
    let mut epd =
        Epd1in54::new(&mut spi, cs_epd, busy_in, dc, rst, &mut delay, None).unwrap();
    epd.sleep(&mut spi, &mut delay).unwrap();

    let flash = Flash::init(spi, cs_flash);
    blink(&mut led);
    let mut flash = flash.unwrap();
    blink(&mut led);
    flash.sleep().unwrap();
    blink(&mut led);

    // release spi1
    let (spi, cs) = flash.release();
    cs.into_pull_up_input();
    let (_, pins) = spi.free();
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
    delay_busy();
    led.set_low().unwrap();
    delay_busy();
}

fn delay_busy() {
    // We can't use `Delay`, as that requires a frequency of at least one MHz.
    // Given our clock selection, the following loop should give us a nice delay
    // when compiled in release mode.
    for _ in 0..100_000 {
        asm::nop()
    }
}
