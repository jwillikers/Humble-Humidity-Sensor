#![no_std]
#![no_main]

use adafruit_feather_rp2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio,
        pac,
        pio::PIOExt,
        spi,
        watchdog::Watchdog,
        I2C,
        Sio,
        Timer,
    },
    Pins,
    XOSC_CRYSTAL_FREQ,
};
use core::convert::TryFrom;
use core::fmt::Write;
use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use epd_waveshare::{
    color::*,
    epd2in9_v2::{Display2in9, Epd2in9},
    graphics::DisplayRotation,
    prelude::*,
};
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle},
    text::{Baseline, Text, TextStyleBuilder},
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::{duration::*, rate::*};
use panic_halt as _;
use shtcx::{self, LowPower, PowerMode};
use smart_leds::{brightness, RGB, SmartLedsWrite};
use smart_leds::hsv::{Hsv, hsv2rgb};
use ws2812_pio::Ws2812;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // todo Save power by using ROSC?
    // let clock = rosc::RingOscillator::new(pac.ROSC).initialize();
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // Configure the on-board NeoPixel.
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    // todo Soft blink?
    let rgb: RGB<u8> = hsv2rgb(Hsv {
        hue: 5,
        sat: 255,
        val: 0,
    });
    let data = [rgb; 1];
    ws.write(data.iter().cloned()).unwrap();

    // Configure the I²C pins
    let sda_pin = pins.sda.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.scl.into_mode::<gpio::FunctionI2C>();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    // These are implicitly used by the SPI driver if they are in the correct mode
    let _spi_sclk = pins.sclk.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.mosi.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.miso.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.d10.into_push_pull_output();

    // Create an SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let dc = pins.d9.into_push_pull_output();
    let rst = pins.d6.into_push_pull_output();
    let busy = pins.d5.into_pull_up_input();

    let mut epd = Epd2in9::new(&mut spi, spi_cs, busy, dc, rst, &mut delay).expect("eink initalize error");
    epd.set_lut(&mut spi, Option::from(RefreshLut::Quick)).unwrap();

    let mut display = Display2in9::default();
    display.set_rotation(DisplayRotation::Rotate270);

    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::iso_8859_1::FONT_9X18_BOLD)
        .text_color(Black)
        .background_color(White)
        .build();
    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style("Humble Humidity Sensor", Point::new(40, 10), style, text_style)
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
        write!(&mut buf, "Temperature: {}°C", measurement.temperature.as_degrees_celsius()).unwrap();
        let _ = Text::with_text_style(buf.as_str(), Point::new(50, 55), style, text_style)
            .draw(&mut display);

        epd.wake_up(&mut spi, &mut delay).unwrap();
        epd
            .update_and_display_frame(&mut spi, display.buffer(), &mut delay)
            .expect("display frame new graphics");
        // todo Implement partial refresh.
        // epd
        //     .update_partial_frame(&mut spi, display.buffer(), 50, 35, 50, 40)
        //     .expect("display frame new graphics");
        // epd.display_frame(&mut spi, &mut delay).unwrap();
        // delay.delay_ms(15_000u32);
        epd.sleep(&mut spi, &mut delay).unwrap();
        delay.delay_ms(Milliseconds::<u32>::try_from(20_u32.minutes()).unwrap().integer());
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
