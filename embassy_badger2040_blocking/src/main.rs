#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{self, Input, Level, Output},
    spi::Spi,
};
use embassy_time::{Delay, Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::{
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};

use uc8151::{Uc8151, LUT, WIDTH};
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    embassy_rp::pac::SIO.spinlock(31).write_value(1);
    info!("Program start");
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut _pwr = Output::new(p.PIN_10, Level::Low);
    let mut _a = Input::new(p.PIN_12, gpio::Pull::Up);
    let mut _b = Input::new(p.PIN_13, gpio::Pull::Up);
    let mut _c = Input::new(p.PIN_14, gpio::Pull::Up);
    let mut _dn = Input::new(p.PIN_11, gpio::Pull::Up);
    let mut _up = Input::new(p.PIN_15, gpio::Pull::Up);
    let reset = Output::new(p.PIN_21, Level::Low);
    let busy = Input::new(p.PIN_26, gpio::Pull::Up);
    let mut dc = Output::new(p.PIN_20, Level::Low);
    let mut cs = Output::new(p.PIN_17, Level::Low);
    let sclk = p.PIN_18;
    let mosi = p.PIN_19;
    let miso = p.PIN_16;

    let mut config = embassy_rp::spi::Config::default();
    config.frequency = 20_000_000;

    let spi = Spi::new(p.SPI0, sclk, mosi, miso, p.DMA_CH0, p.DMA_CH1, config);
    dc.set_high();
    cs.set_high();
    let mut display = Uc8151::new(spi, cs, dc, busy, reset);

    display.disable();
    Timer::after(Duration::from_millis(10)).await;
    display.enable();
    Timer::after(Duration::from_millis(10)).await;
    while display.is_busy() {}

    let _ = display.setup(&mut Delay, LUT::Internal);

    let text = "Hello\nfrom\nRust!";
    // Note we're setting the Text color to `Off` - this is Black in the e-ink display driver
    let character_style = MonoTextStyle::new(&FONT_9X18_BOLD, BinaryColor::Off);
    let textbox_style = TextBoxStyleBuilder::new()
        .height_mode(HeightMode::FitToText)
        .alignment(HorizontalAlignment::Center)
        .paragraph_spacing(6)
        .build();
    // Bounding box for our text. Fill it with the opposite color so we can read the text.
    let bounds = Rectangle::new(Point::new(157, 10), Size::new(WIDTH - 157, 0));
    bounds
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(&mut display)
        .unwrap();
    // Create the text box and apply styling options.
    let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);
    // Clear the rest of the screen
    display.clear(BinaryColor::On).unwrap();
    // Draw the text box.
    text_box.draw(&mut display).unwrap();
    // Transfer the framebuffer and then refresh the screen
    display.update().unwrap();

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(Duration::from_secs(1)).await;

        info!("led off!");
        led.set_low();
        Timer::after(Duration::from_secs(1)).await;
    }
}
