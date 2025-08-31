//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use rp235x_hal::clocks::init_clocks_and_plls;
use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::{self as hal, entry};
use rp235x_hal::{Clock, pac};

use embedded_graphics::{
    geometry::AnchorPoint,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_9X15},
    pixelcolor::Rgb565,
    prelude::{DrawTarget, RgbColor},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
// use embedded_hal::digital::v2::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::{models::ST7789, options::ColorInversion};
use panic_probe as _;

use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_hal::delay::DelayNs;
use mipidsi::interface::SpiInterface;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use some_bsp;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = cortex_m::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico 2 board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico 2 W, the LED is not connected to any of the RP2350 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Setup SPI
    let display_dc = pins.gpio16.into_push_pull_output();
    let display_cs = pins.gpio17.into_push_pull_output();
    let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();

    let spi_bus = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi_bus = spi_bus.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        12_000_000u32.Hz(),
        embedded_hal::spi::MODE_3,
    );
    let excl_spi_dev = ExclusiveDevice::new(spi_bus, display_cs, timer).unwrap();
    let mut rst_pin = pins.gpio22.into_push_pull_output();
    rst_pin.set_low().unwrap();

    let mut buffer = [0_u8; 512];
    let di = SpiInterface::new(excl_spi_dev, display_dc, &mut buffer);
    let orientation = Orientation::new().rotate(Rotation::Deg90);

    let mut display = mipidsi::Builder::new(ST7789, di)
        .display_size(135u16, 240u16)
        .display_offset(52, 40)
        .invert_colors(ColorInversion::Inverted)
        .reset_pin(rst_pin)
        .orientation(orientation)
        .init(&mut timer)
        .unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    const Y_MAX: i32 = 134;
    const X_MAX: i32 = 239;
    const Y_MIN: i32 = 1;
    const X_MIN: i32 = 1;

    const CIRCLE_WIDTH: u32 = 60;
    const CIRCLE_RADIUS: i32 = 30;

    let circle_origin_red = Circle::new(
        Point::new(0 - CIRCLE_RADIUS, 0 - CIRCLE_RADIUS),
        CIRCLE_WIDTH,
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::RED));

    let circle_xmax_blue = Circle::new(
        Point::new(X_MAX - CIRCLE_RADIUS, 0 - CIRCLE_RADIUS),
        CIRCLE_WIDTH,
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE));

    let circle_ymax_green = Circle::new(
        Point::new(0 - CIRCLE_RADIUS, Y_MAX - CIRCLE_RADIUS),
        CIRCLE_WIDTH,
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN));

    let circle_xmax_ymax_yellow = Circle::new(
        Point::new(X_MAX - CIRCLE_RADIUS, Y_MAX - CIRCLE_RADIUS),
        CIRCLE_WIDTH,
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::YELLOW));

    let line_origin_to_xmax_ymax_purple =
        Line::new(Point::new(X_MIN, Y_MIN), Point::new(X_MAX, Y_MAX))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::CSS_PURPLE, 1));

    let line_xmax_0_to_0_ymax_white = Line::new(Point::new(X_MAX, Y_MIN), Point::new(X_MIN, Y_MAX))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1));

    display.clear(Rgb565::WHITE).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    circle_origin_red.draw(&mut display).unwrap();
    circle_xmax_blue.draw(&mut display).unwrap();
    circle_ymax_green.draw(&mut display).unwrap();
    circle_xmax_ymax_yellow.draw(&mut display).unwrap();
    line_origin_to_xmax_ymax_purple.draw(&mut display).unwrap();
    line_xmax_0_to_0_ymax_white.draw(&mut display).unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(Rgb565::WHITE)
        .build();

    let center_aligned = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Middle)
        .build();

    let bb = display.bounding_box().offset(-20);
    for anchor in [
        AnchorPoint::TopLeft,
        AnchorPoint::TopCenter,
        AnchorPoint::TopRight,
        AnchorPoint::CenterLeft,
        AnchorPoint::Center,
        AnchorPoint::CenterRight,
        AnchorPoint::BottomLeft,
        AnchorPoint::BottomCenter,
        AnchorPoint::BottomRight,
    ]
    .into_iter()
    {
        Text::with_text_style("ABC", bb.anchor_point(anchor), text_style, center_aligned)
            .draw(&mut display)
            .unwrap();
    }

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        timer.delay_ns(500_000_000);
        info!("off!");
        led_pin.set_low().unwrap();
        timer.delay_ns(500_000_000);
    }
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 Template"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

// End of file
