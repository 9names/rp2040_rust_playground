//! Simple test of pimoroni unicorn PIO library
//!
//!
//!
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;
use pimoroni_unicorn_pio::UnicornPins;

use bsp::hal::{clocks::init_clocks_and_plls, pac, pio::PIOExt, sio::Sio, watchdog::Watchdog};
use rp_pico as bsp;

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
    let mut btn_a = pins.gpio12.into_floating_input();
    let mut btn_b = pins.gpio13.into_floating_input();
    let mut btn_x = pins.gpio14.into_floating_input();
    let mut btn_y = pins.gpio15.into_floating_input();

    let loop_update_freq = 1000;
    let mut loop_counter = 0;

    let mut led_pin = pins.led.into_push_pull_output();

    let _ = led_pin.toggle();
    loop {
        let colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255)];
        for color in colours {
            for y in 0..pimoroni_unicorn_pio::HEIGHT as u8 {
                for x in 0..pimoroni_unicorn_pio::WIDTH as u8 {
                    uni.set_pixel(x, y, color.0, color.1, color.2);
                    uni.draw();
                    uni.set_pixel(x, y, 0, 0, 0);
                }
            }
        }
    }
}

// End of file
