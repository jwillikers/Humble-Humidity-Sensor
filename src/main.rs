#![no_std]
#![no_main]

use bsp::hal;
use bsp::pac;
use qt_py_m0 as bsp;

use bsp::{entry, pin_alias};
use hal::clock::{enable_internal_32kosc, ClockGenId, ClockSource, GenericClockController};
use hal::prelude::*;
use hal::rtc;
use hal::sleeping_delay::SleepingDelay;

use core::convert::TryFrom;
use core::fmt::Write;
use core::sync::atomic;
use cortex_m::peripheral::NVIC;
use pac::{interrupt, CorePeripherals, Peripherals, RTC};

use crate::hal::sercom::{i2c, spi};
use crate::hal::time::{KiloHertz, MegaHertz};
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle},
    text::{Baseline, Text, TextStyleBuilder},
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::{duration::*, rate::*};
use epd_waveshare::{
    color::*,
    epd2in9_v2::{Display2in9, Epd2in9},
    graphics::DisplayRotation,
    prelude::*,
};
use qt_py_m0::hal::gpio;
use hal::delay::Delay;
use panic_halt as _;
use qt_py_m0::{I2c, Spi};
use shtcx::{self, LowPower, PowerMode};

/// Shared atomic between RTC interrupt and sleeping_delay module
static INTERRUPT_FIRED: atomic::AtomicBool = atomic::AtomicBool::new(false);

#[entry]
fn main() -> ! {
    // Configure all of our peripherals/clocks
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_8mhz(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Get a clock & make a sleeping delay object. use internal 32k clock that runs
    // in standby
    enable_internal_32kosc(&mut peripherals.SYSCTRL);
    let timer_clock = clocks
        .configure_gclk_divider_and_source(ClockGenId::GCLK1, 1, ClockSource::OSC32K, false)
        .unwrap();
    clocks.configure_standby(ClockGenId::GCLK1, true);
    let rtc_clock = clocks.rtc(&timer_clock).unwrap();
    let timer = rtc::Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);
    let mut sleeping_delay = SleepingDelay::new(timer, &INTERRUPT_FIRED);

    // We can use the RTC in standby for maximum power savings
    core.SCB.set_sleepdeep();

    // enable interrupts
    unsafe {
        core.NVIC.set_priority(interrupt::RTC, 2);
        NVIC::unmask(interrupt::RTC);
    }

    let mut pm = peripherals.PM;

    // Turn off unnecessary peripherals
    pm.ahbmask.modify(|_, w| {
        w.usb_().clear_bit();
        w.dmac_().clear_bit()
    });
    pm.apbamask.modify(|_, w| {
        w.eic_().clear_bit();
        w.wdt_().clear_bit();
        w.sysctrl_().clear_bit();
        w.pac0_().clear_bit()
    });
    pm.apbbmask.modify(|_, w| {
        w.usb_().clear_bit();
        w.dmac_().clear_bit();
        w.nvmctrl_().clear_bit();
        w.dsu_().clear_bit();
        w.pac1_().clear_bit()
    });

    // Thankfully the only one default on here is ADC
    pm.apbcmask.modify(|_, w| w.adc_().clear_bit());

    let pins = bsp::Pins::new(peripherals.PORT).split();

    // Configure the I²C pins
    // let gclk0 = clocks.gclk0();
    let mut i2c = pins.i2c.init(&mut clocks, KiloHertz(100), peripherals.SERCOM1, &mut pm);

    // These are implicitly used by the SPI driver if they are in the correct mode
    let spi_cs = pins.analog.a3.into_push_pull_output();

    let mut spi = pins.spi.init(
        &mut clocks,
        MegaHertz(10),
        peripherals.SERCOM2,
        &mut pm,
    );

    let dc = pins.analog.a2.into_push_pull_output();
    let rst = pins.analog.a1.into_push_pull_output();
    let busy = pins.analog.a0.into_pull_up_input();

    let mut epd = Epd2in9::new(&mut spi, spi_cs, busy, dc, rst, &mut sleeping_delay)
        .expect("eink initalize error");
    epd.set_lut(&mut spi, Option::from(RefreshLut::Quick))
        .unwrap();

    let mut display = Display2in9::default();
    display.set_rotation(DisplayRotation::Rotate270);

    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::iso_8859_1::FONT_9X18_BOLD)
        .text_color(Black)
        .background_color(White)
        .build();
    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style(
        "Humble Humidity Sensor",
        Point::new(40, 10),
        style,
        text_style,
    )
    .draw(&mut display);

    let mut sht = shtcx::shtc3(i2c);

    let mut buf = FmtBuf::new();

    // epd.update_frame(&mut spi, display.buffer(), &mut delay).unwrap();
    // epd
    //     .display_frame(&mut spi, &mut delay)
    //     .expect("display frame new graphics");
    // epd.set_refresh(&mut spi, &mut delay, RefreshLut::Quick)
    //     .unwrap();

    loop {
        sht.wakeup(&mut delay).unwrap();
        let measurement = sht.measure(PowerMode::LowPower, &mut delay).unwrap();
        sht.sleep().unwrap();

        buf.reset();
        write!(&mut buf, "Humidity: {}%", measurement.humidity.as_percent()).unwrap();
        let _ = Text::with_text_style(buf.as_str(), Point::new(50, 35), style, text_style)
            .draw(&mut display);

        buf.reset();
        write!(
            &mut buf,
            "Temperature: {}°C",
            measurement.temperature.as_degrees_celsius()
        )
        .unwrap();
        let _ = Text::with_text_style(buf.as_str(), Point::new(50, 55), style, text_style)
            .draw(&mut display);

        epd.wake_up(&mut spi, &mut sleeping_delay).unwrap();
        epd.update_and_display_frame(&mut spi, display.buffer(), &mut sleeping_delay)
            .expect("display frame new graphics");
        // todo Implement partial refresh.
        // epd
        //     .update_partial_frame(&mut spi, display.buffer(), 50, 35, 50, 40)
        //     .expect("display frame new graphics");
        // epd.display_frame(&mut spi, &mut delay).unwrap();
        // delay.delay_ms(15_000u32);
        epd.sleep(&mut spi, &mut sleeping_delay).unwrap();
        sleeping_delay.delay_ms(
            Milliseconds::<u32>::try_from(20_u32.minutes())
                .unwrap()
                .integer(),
        );
    }
}

/// The FmtBuf class is from the rp-pico example pico_i2c_oled_display_ssd1306.
/// https://github.com/rp-rs/rp-hal/blob/main/boards/rp-pico/examples/pico_i2c_oled_display_ssd1306.rs
/// It allows formatting output without requiring an allocator to be configured for the project.
/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 64 bytes.
struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            ptr: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}

#[interrupt]
fn RTC() {
    // Let the sleepingtimer know that the interrupt fired, and clear it
    INTERRUPT_FIRED.store(true, atomic::Ordering::Relaxed);
    unsafe {
        RTC::ptr()
            .as_ref()
            .unwrap()
            .mode0()
            .intflag
            .modify(|_, w| w.cmp0().set_bit());
    }
}
