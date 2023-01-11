#![no_main]
#![no_std]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use cortex_m::{asm, interrupt, peripheral::NVIC};

use cortex_m_rt::entry;
// use cortex_m_semihosting::hprintln;
use spi_memory::series25::Flash;
use stm32l0xx_hal::{
    adc::Adc,
    delay::Delay,
    exti::{ConfigurableLine, Exti, ExtiLine, GpioLine, TriggerEdge},
    flash::FLASH,
    gpio::{
        gpiob::{PB13, PB14, PB15, PB8},
        Analog, Output, Pin, PushPull,
    },
    pac::{self, SPI2},
    prelude::*,
    pwr::{self, PWR},
    rcc::{self, SMEnable},
    rtc::{self, ClockSource, Rtc},
    spi::{Spi, MODE_0, MODE_3},
    syscfg::SYSCFG,
};

// For GDE015OC1 use:
// use epd_waveshare::{epd1in54::*, prelude::*};
// For GDEH0154D67 use:
use epd_waveshare::{epd1in54_v2::*, prelude::*};

use lis3dh_spi::{
    ctrl_reg_1_value::{CtrlReg1Value, LPEn, XEn, YEn, ZEn, ODR},
    ctrl_reg_2_value::{CtrlReg2Value, FilteredDataSelection, HighPassFilter},
    ctrl_reg_3_value::CtrlReg3Value,
    ctrl_reg_4_value::{CtrlReg4Value, FullScaleSelection},
    ctrl_reg_5_value::CtrlReg5Value,
    enabled_enum::OnOff,
    int_cfg::IntCfg,
    int_duration_value::IntDuration,
    int_ths_value::IntThs,
};

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
    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    pac::SYSCFG::disable_in_sleep_mode(&mut rcc);
    let mut exti = Exti::new(dp.EXTI);
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let mut delay = Delay::new(cp.SYST, rcc.clocks);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut solar_in = gpioa.pa0.into_analog();
    let _ = gpioa.pa1.into_analog();
    let accel_int1 = gpioa.pa2.into_floating_input();
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

    let _nvm = FLASH::new(dp.FLASH, &mut rcc);

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
    // a read too soon after Adc::new() can freeze
    delay.delay_ms(1u32);
    let _dummy_read: u16 = adc.read(&mut solar_in).unwrap();

    // wait for accel to boot
    delay.delay_ms(1u32);

    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), MODE_0, 2_000_000.Hz(), &mut rcc);

    let mut spi2 = dp
        .SPI2
        .spi((sck2, miso2, mosi2), MODE_3, 2_000_000.Hz(), &mut rcc);

    let mut accelerometer = lis3dh_spi::Lis3dh::default();
    accel_config(&mut accelerometer, &mut cs_accel, &mut spi2);
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

    // leave accelerator on
    accel_off.into_pull_down_input();

    // put display to sleep
    let mut epd = Epd1in54::new(&mut spi, cs_epd, busy_in, dc, rst, &mut delay, None).unwrap();
    epd.set_lut(&mut spi, &mut delay, Some(RefreshLut::Full))
        .unwrap();
    epd.clear_frame(&mut spi, &mut delay).unwrap();
    epd.display_frame(&mut spi, &mut delay).unwrap();
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
    let rtc_line = ConfigurableLine::RtcWakeup;
    rtc.enable_interrupts(rtc::Interrupts {
        wakeup_timer: true,
        ..rtc::Interrupts::default()
    });
    exti.listen_configurable(rtc_line, TriggerEdge::Rising);
    let rtc_interrupt = rtc_line.interrupt();

    let mut timer = rtc.wakeup_timer();

    // 20 seconds of stop mode
    timer.start(20u32);

    // To use solar_in as an interrupt, convert from analog to digital input
    let solar_in = solar_in.into_floating_input();
    let solar_line = GpioLine::from_raw_line(solar_in.pin_number()).unwrap();
    exti.listen_gpio(
        &mut syscfg,
        solar_in.port(),
        solar_line,
        TriggerEdge::Rising,
    );
    let solar_interrupt = solar_line.interrupt();

    // Wake up on accelerometer gestures
    let accel_line = GpioLine::from_raw_line(accel_int1.pin_number()).unwrap();
    exti.listen_gpio(
        &mut syscfg,
        accel_int1.port(),
        accel_line,
        TriggerEdge::Rising,
    );
    let accel_interrupt = accel_line.interrupt();

    // Disable all gpio clocks
    rcc.iopenr.modify(|_, w| w.iopaen().disabled());
    rcc.iopenr.modify(|_, w| w.iopben().disabled());
    rcc.iopenr.modify(|_, w| w.iopcen().disabled());
    rcc.iopenr.modify(|_, w| w.iopden().disabled());
    rcc.iopenr.modify(|_, w| w.iopeen().disabled());
    rcc.iopenr.modify(|_, w| w.iophen().disabled());

    // configure rcc to disable all these when entering
    // stop mode
    rcc.apb1smenr.modify(|_, w| w.spi2smen().disabled());
    rcc.apb2smenr.modify(|_, w| w.spi1smen().disabled());
    rcc.apb2smenr.modify(|_, w| w.adcsmen().disabled());
    rcc.ahbsmenr.modify(|_, w| w.mifsmen().disabled());
    let adc = adc.release();
    adc.cr.modify(|_, w| w.aden().clear_bit());
    adc.cr.modify(|_, w| w.advregen().clear_bit());

    // hprintln!("apb1smenr: {:08x}", rcc.apb1smenr.read().bits());
    // hprintln!("apb2smenr: {:08x}", rcc.apb2smenr.read().bits());
    // hprintln!("ahbsmenr: {:08x}", rcc.ahbsmenr.read().bits());
    // hprintln!("adc.cr: {:08x}", adc.cr.read().bits());

    loop {
        interrupt::free(|_| {
            unsafe {
                NVIC::unmask(solar_interrupt);
                NVIC::unmask(rtc_interrupt);
                NVIC::unmask(accel_interrupt);
            }
            pwr.stop_mode(
                &mut scb,
                &mut rcc,
                pwr::StopModeConfig {
                    ultra_low_power: true,
                },
            )
            .enter();

            if Exti::is_pending(solar_line) {
                Exti::unpend(solar_line);
                NVIC::unpend(solar_interrupt);
                NVIC::mask(solar_interrupt);
            }
            if Exti::is_pending(accel_line) {
                Exti::unpend(accel_line);
                NVIC::unpend(accel_interrupt);
                NVIC::mask(accel_interrupt);
            }
            if Exti::is_pending(rtc_line) {
                // next call will return immediately, as it was the timer interrupt that
                // woke us up
                timer.wait().unwrap();
                Exti::unpend(rtc_line);
                NVIC::unpend(rtc_interrupt);
                NVIC::mask(rtc_interrupt);
            }
        });

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

// Configure the accelerator after a power-up or reset
fn accel_config(
    accelerometer: &mut lis3dh_spi::Lis3dh,
    cs: &mut PB8<Output<PushPull>>,
    spi: &mut Spi<SPI2, (PB13<Analog>, PB14<Analog>, PB15<Analog>)>,
) -> () {
    // Note: The pull-up disable will not persist: it returns back
    // to enabled.  So disabling this or the accel configuration consistency
    // check will fail.
    //
    // Disable internal pull-up on MISO pin
    // let mut accel_cfg0 = CtrlReg0Value::default();
    // // The semantics of this function is revered:  Enabled means disable pull-up!!
    // accel_cfg0.set_pull_up_connected_sdo_sa_0_pin(OnOff::Enabled);
    // accelerometer.set_ctrl_reg0_setting(accel_cfg0);

    // Configuring accelerometer wakeup interrupt
    // per AN5005, section 6.3.3
    let mut accel_cfg1 = CtrlReg1Value::default();
    accel_cfg1.set_x_en(XEn::XAxisEnabled);
    accel_cfg1.set_y_en(YEn::YAxisEnabled);
    accel_cfg1.set_z_en(ZEn::ZAxisEnabled);
    accel_cfg1.set_output_data_rate(ODR::Hz100);
    // Lowers resolution to 8-bits for lower power consumption (10 uA at 100Hz)
    accel_cfg1.set_l_p_en(LPEn::LowPowerEnabled);
    accelerometer.set_ctrl_reg1_setting(accel_cfg1);

    // Enable high pass filter to make gestures independent of static
    // orientation of device.  (i.e. otherwise, threshold would be
    // constantly exceeded when device is laying on one of its sides)
    let mut accel_cfg2 = CtrlReg2Value::default();
    accel_cfg2.set_hp_ia1(HighPassFilter::FilterEnabled);
    accel_cfg2.set_fds(FilteredDataSelection::InternalFilterSentToFifo);
    accelerometer.set_ctrl_reg2_setting(accel_cfg2);

    // Enable INT1
    let mut accel_cfg3 = CtrlReg3Value::default();
    accel_cfg3.set_interrupt_1_ia1(OnOff::Enabled);
    accelerometer.set_ctrl_reg3_setting(accel_cfg3);

    // Set full range to 2g
    let mut accel_cfg4 = CtrlReg4Value::default();
    accel_cfg4.set_fs(FullScaleSelection::Gravity2G);
    accelerometer.set_ctrl_reg4_setting(accel_cfg4);

    // Interrupt 1 pin NOT latched, so no need to clear
    let mut accel_cfg5 = CtrlReg5Value::default();
    accel_cfg5.set_latch_int_on_int_1_src(OnOff::Disabled);
    accelerometer.set_ctrl_reg5_setting(accel_cfg5);

    let mut intths = IntThs::default();
    // 250 mg threshold
    // intths.set_threshold(0x10).unwrap();
    // 1g threshold
    intths.set_threshold(0x40).unwrap();
    accelerometer.set_int1_ths_setting(intths);

    let mut intduration = IntDuration::default();
    // duration is in ODRs, which at 100Hz corresponds to 1/100s
    // this is the time the acceleration needs to remain above
    // threshold to trigger an interrupt
    intduration.set_duration(0).unwrap();
    accelerometer.set_int1_duration_setting(intduration);

    // this pushes all the configuration registers configured so far to
    // the device
    accelerometer.write_all_settings(cs, spi).ok();

    // dummy read to force HP filter to load current acceleration
    // values
    let _ = accelerometer.get_reference_value(cs, spi);

    // Enable high threshold value on X, Y only
    let mut intcfg = IntCfg::default();
    intcfg.set_xhie(OnOff::Enabled);
    intcfg.set_yhie(OnOff::Enabled);
    intcfg.set_aoi(OnOff::Disabled);
    accelerometer.set_int1_cfg_setting(intcfg);
    accelerometer.write_all_settings(cs, spi).ok();
}
