//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_graphics::pixelcolor::Rgb888;
use panic_probe as _;
use rp235x_hal::clocks::init_clocks_and_plls;
use rp235x_hal::pac;
use rp235x_hal::pio::PIOExt;
use rp235x_hal::{self as hal, Sio, Watchdog, entry};

use embedded_graphics::prelude::RgbColor;

use panic_probe as _;

use embedded_graphics::prelude::*;

use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use pimoroni_unicorn_pio_rp2350::UnicornPins;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

struct TogglePin<T: embedded_hal::digital::OutputPin> {
    pin: T,
    state: embedded_hal::digital::PinState,
}

impl<T> TogglePin<T>
where
    T: embedded_hal::digital::OutputPin,
{
    fn toggle(&mut self) {
        self.state = match self.state {
            rp235x_hal::gpio::PinState::Low => rp235x_hal::gpio::PinState::High,
            rp235x_hal::gpio::PinState::High => rp235x_hal::gpio::PinState::Low,
        };
        self.pin.set_state(self.state).unwrap();
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let unipins = UnicornPins {
        led_blank: pins.gpio11.into_function().into_pull_type().into_dyn_pin(),
        led_latch: pins.gpio10.into_function().into_pull_type().into_dyn_pin(),
        led_clock: pins.gpio9.into_function().into_pull_type().into_dyn_pin(),
        led_data: pins.gpio8.into_function().into_pull_type().into_dyn_pin(),
        row_0: pins.gpio22.into_function().into_pull_type().into_dyn_pin(),
        row_1: pins.gpio21.into_function().into_pull_type().into_dyn_pin(),
        row_2: pins.gpio20.into_function().into_pull_type().into_dyn_pin(),
        row_3: pins.gpio19.into_function().into_pull_type().into_dyn_pin(),
        row_4: pins.gpio18.into_function().into_pull_type().into_dyn_pin(),
        row_5: pins.gpio17.into_function().into_pull_type().into_dyn_pin(),
        row_6: pins.gpio16.into_function().into_pull_type().into_dyn_pin(),
    };

    let mut uni = pimoroni_unicorn_pio_rp2350::Unicorn::new(&mut pio, sm0, unipins);
    // Buttons
    let mut _btn_a = pins.gpio12.into_floating_input();
    let mut _btn_b = pins.gpio13.into_floating_input();
    let mut _btn_x = pins.gpio14.into_floating_input();
    let mut _btn_y = pins.gpio15.into_floating_input();

    let mut led_pin = TogglePin {
        pin: pins.gpio25.into_push_pull_output(),
        state: embedded_hal::digital::PinState::Low,
    };

    led_pin.toggle();
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
            for y in 0..pimoroni_unicorn_pio_rp2350::HEIGHT as i32 {
                for x in 0..pimoroni_unicorn_pio_rp2350::WIDTH as i32 {
                    uni.set_pixel(Point::new(x, y), colour);
                    uni.draw();
                    uni.set_pixel(Point::new(x, y), clear);
                }
            }
        }
    }
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 unicorn"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

// End of file
