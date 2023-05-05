//! Pico + 8x8 ws2812(neopixel) RGB LED array
//!
//! This demo showcases driving an 8x8 RGB LED array connected to GPIO22
//! It's a slightly modified version of the example from the Pico repo
//! https://github.com/rp-rs/rp-hal/blob/main/boards/rp-pico/examples/pico_ws2812_led.rs
//!
//! This code should be compatible with any ws2812 panel - I used https://core-electronics.com.au/glowbit-matrix-8x8.html
//! Adjust STRIP_LEN to match the number of LEDs in your ws2812 array
#![no_std]
#![no_main]

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_graphics::image::Image;
use embedded_graphics::image::ImageRawLE;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::RateExtU32;
use panic_probe as _;
use st7789::{Orientation, ST7789};
// Expose our board support package as `bsp` to make porting code between boards easier
use rp_pico as bsp;

struct RgbPins {
    pub red: hal::gpio::Pin<hal::gpio::bank0::Gpio6, hal::gpio::Output<hal::gpio::PushPull>>,
    pub green: hal::gpio::Pin<hal::gpio::bank0::Gpio7, hal::gpio::Output<hal::gpio::PushPull>>,
    pub blue: hal::gpio::Pin<hal::gpio::bank0::Gpio8, hal::gpio::Output<hal::gpio::PushPull>>,
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
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

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    // We're using what would normally be the miso pin for data/command
    let dc = pins.gpio16.into_push_pull_output();
    // Chip select
    let cs = pins.gpio17.into_push_pull_output();
    // Backlight
    let bl = pins.gpio21.into_push_pull_output();

    // All the pushbuttons
    let button_a = pins.gpio12.into_pull_up_input();
    let button_b = pins.gpio13.into_pull_up_input();
    let button_x = pins.gpio14.into_pull_up_input();
    let button_y = pins.gpio15.into_pull_up_input();

    let mut rgb: RgbPins = RgbPins {
        red: pins.gpio6.into_push_pull_output(),
        green: pins.gpio7.into_push_pull_output(),
        blue: pins.gpio8.into_push_pull_output(),
    };

    debug!("setting up spi");
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    // create driver
    let di = SPIInterface::new(spi, dc, cs);
    let mut display = ST7789::new(di, None, Some(bl), 240, 135);

    debug!("initialising display");
    // initialize
    display.init(&mut delay).unwrap();
    // set default orientation
    display.set_orientation(Orientation::Landscape).unwrap();

    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(60, 60));

    // draw image on black background
    display.clear(Rgb565::BLACK).unwrap();
    ferris.draw(&mut display).unwrap();

    // Turn all the elements of the RGB LED off
    rgb.red.set_high().unwrap();
    rgb.green.set_high().unwrap();
    rgb.blue.set_high().unwrap();
    // Start with the Pico LED on, so that it is toggled out of phase with the Display LED
    led_pin.set_high().unwrap();
    debug!("entering main loop");
    loop {
        // Update the state of the RGB LED to indicate that a button press has been observed
        if button_a.is_low().unwrap() {
            rgb.red.set_low().unwrap();
            rgb.green.set_high().unwrap();
        } else if button_b.is_low().unwrap() {
            rgb.green.set_low().unwrap();
            rgb.red.set_high().unwrap();
        } else if button_x.is_low().unwrap() {
            rgb.green.set_low().unwrap();
            rgb.red.set_low().unwrap();
        } else if button_y.is_low().unwrap() {
            rgb.green.set_low().unwrap();
            rgb.red.set_low().unwrap();
            rgb.blue.set_high().unwrap();
        } else {
            rgb.red.set_high().unwrap();
            rgb.green.set_high().unwrap();
        }
        // Toggle the Pico's led so we can show the code is working,
        led_pin.toggle().unwrap();
        // Toggle the Pico display's blue LED so we can show that is working too
        rgb.blue.toggle().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
