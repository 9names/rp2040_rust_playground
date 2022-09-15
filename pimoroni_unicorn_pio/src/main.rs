//! Simple test of pimoroni unicorn PIO library
//!
//!
//!
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics_core::pixelcolor::Rgb888;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;
use pimoroni_unicorn_pio::UnicornPins;

use bsp::hal::{clocks::init_clocks_and_plls, pac, pio::PIOExt, sio::Sio, watchdog::Watchdog};
use rp_pico as bsp;

use embedded_graphics::{
    pixelcolor::RgbColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _clocks = init_clocks_and_plls(
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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let unipins = UnicornPins {
        led_blank: pins.gpio11.into_mode(),
        led_latch: pins.gpio10.into_mode(),
        led_clock: pins.gpio9.into_mode(),
        led_data: pins.gpio8.into_mode(),
        row_0: pins.gpio22.into_mode(),
        row_1: pins.gpio21.into_mode(),
        row_2: pins.gpio20.into_mode(),
        row_3: pins.gpio19.into_mode(),
        row_4: pins.gpio18.into_mode(),
        row_5: pins.gpio17.into_mode(),
        row_6: pins.gpio16.into_mode(),
    };

    let mut uni = pimoroni_unicorn_pio::Unicorn::new(&mut pio, sm0, unipins);
    // Buttons
    let mut _btn_a = pins.gpio12.into_floating_input();
    let mut _btn_b = pins.gpio13.into_floating_input();
    let mut _btn_x = pins.gpio14.into_floating_input();
    let mut _btn_y = pins.gpio15.into_floating_input();

    let mut led_pin = pins.led.into_push_pull_output();

    let _ = led_pin.toggle();
    Rectangle::new(Point::new(1, 5), Size::new(2, 3))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(Rgb888::RED)
                .stroke_width(5)
                .fill_color(Rgb888::GREEN)
                .build(),
        )
        .draw(&mut uni)
        .unwrap();
    loop {
        let colours = [
            Rgb888::new(255, 0, 0),
            Rgb888::new(0, 255, 0),
            Rgb888::new(0, 0, 255),
        ];
        let clear = Rgb888::new(0, 0, 0);
        for colour in colours {
            for y in 0..pimoroni_unicorn_pio::HEIGHT as i32 {
                for x in 0..pimoroni_unicorn_pio::WIDTH as i32 {
                    uni.set_pixel(Point::new(x, y), colour);
                    uni.draw();
                    uni.set_pixel(Point::new(x, y), clear);
                }
            }
        }
    }
}

// End of file
