#![no_std]
#![no_main]

use bsp::{entry, hal};
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::{
    geometry::AnchorPoint,
    mono_font::{ascii::FONT_9X15, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::{DrawTarget, RgbColor},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use embedded_hal::digital::v2::OutputPin;
use mipidsi::{Builder, Orientation};
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use display_interface_spi::SPIInterface;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use fugit::RateExtU32;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    // Setup SPI
    let spi_cs = pins.gpio17.into_push_pull_output();
    let spi_miso = pins.gpio16.into_push_pull_output();
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        12_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let mut rst_pin = pins.gpio22.into_push_pull_output();
    rst_pin.set_low().unwrap();

    let di = SPIInterface::new(spi, spi_miso, spi_cs);
    let mut display = Builder::st7789_pico1(di)
        .with_display_size(135, 240)
        .with_orientation(Orientation::Landscape(true))
        .init(&mut delay, Some(rst_pin))
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
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
