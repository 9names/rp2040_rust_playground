//! Simple test of the pimoroni unicorn - bitbang each row of lights.
//!
//! Only allows each LED to be off or on, so there is only 8 possible colours: black white red green blue cyan magenta yellow
//! This program exists to show how data is clocked into the device - we could use duty cycle to approximate intermediate colours
//! but it would be better to use the PIO program that Pimoroni wrote for this purpose.
//! LED scanning is based on the PIO program anyway:
//! https://github.com/pimoroni/pimoroni-pico/blob/main/libraries/pico_unicorn/pico_unicorn.pio
//!
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;

use panic_probe as _;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico as bsp;

#[derive(Copy, Clone)]
struct Rgb {
    red: bool,
    blue: bool,
    green: bool,
}

const COL_COUNT: u32 = 16;
const ROW_COUNT: u32 = 7;
const ARRAY_SIZE: usize = (ROW_COUNT * COL_COUNT) as usize;
static mut FRAME_BUFFER: [Rgb; ARRAY_SIZE] = [Rgb {
    red: false,
    blue: false,
    green: false,
}; ARRAY_SIZE];

static mut CURRENT_COLOUR: u8 = 0;
static mut CURRENT_PIXEL: u16 = 0;

fn lookup_colour(index: u8) -> Rgb {
    match index {
        0 => Rgb {
            red: true,
            green: false,
            blue: false,
        },
        1 => Rgb {
            red: false,
            green: true,
            blue: false,
        },
        2 => Rgb {
            red: false,
            green: false,
            blue: true,
        },
        3 => Rgb {
            red: true,
            green: true,
            blue: false,
        },
        4 => Rgb {
            red: false,
            green: true,
            blue: true,
        },
        5 => Rgb {
            red: true,
            green: false,
            blue: true,
        },
        6 => Rgb {
            red: true,
            green: true,
            blue: true,
        },
        _ => Rgb {
            red: false,
            green: false,
            blue: false,
        },
    }
}

fn update_framebuffer() {
    let mut colour = unsafe { CURRENT_COLOUR };
    let mut pixel = unsafe { CURRENT_PIXEL };
    (0..ARRAY_SIZE as usize).for_each(|index| {
        let colour = if pixel == index as u16 {
            lookup_colour(colour)
        } else {
            lookup_colour(7)
        };
        unsafe { FRAME_BUFFER[index] = colour };
    });
    pixel += 1;
    if pixel >= ARRAY_SIZE as u16 {
        pixel = 0;
        colour += 1;
    }
    if colour >= 7 {
        colour = 0;
    }
    unsafe {
        CURRENT_PIXEL = pixel;
        CURRENT_COLOUR = colour;
    }
}

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

    // LED pins
    let mut led_data = pins.gpio8.into_push_pull_output();
    let mut led_clock = pins.gpio9.into_push_pull_output();
    let mut led_latch = pins.gpio10.into_push_pull_output();
    let mut led_blank = pins.gpio11.into_push_pull_output();
    let mut row_0 = pins.gpio22.into_push_pull_output();
    let mut row_1 = pins.gpio21.into_push_pull_output();
    let mut row_2 = pins.gpio20.into_push_pull_output();
    let mut row_3 = pins.gpio19.into_push_pull_output();
    let mut row_4 = pins.gpio18.into_push_pull_output();
    let mut row_5 = pins.gpio17.into_push_pull_output();
    let mut row_6 = pins.gpio16.into_push_pull_output();

    // Buttons
    let mut btn_a = pins.gpio12.into_floating_input();
    let mut btn_b = pins.gpio13.into_floating_input();
    let mut btn_x = pins.gpio14.into_floating_input();
    let mut btn_y = pins.gpio15.into_floating_input();

    let loop_update_freq = 1000;
    let mut loop_counter = 0;

    led_blank.set_low().unwrap();
    led_latch.set_low().unwrap();
    loop {
        loop_counter += 1;
        if loop_counter >= loop_update_freq {
            update_framebuffer();
            loop_counter = 0;
        }
        for row in 0..ROW_COUNT {
            row_6.set_high().unwrap();
            row_5.set_high().unwrap();
            row_4.set_high().unwrap();
            row_3.set_high().unwrap();
            row_2.set_high().unwrap();
            row_1.set_high().unwrap();
            row_0.set_high().unwrap();
            match row {
                6 => {
                    row_6.set_low().unwrap();
                }
                5 => {
                    row_5.set_low().unwrap();
                }
                4 => {
                    row_4.set_low().unwrap();
                }
                3 => {
                    row_3.set_low().unwrap();
                }
                2 => {
                    row_2.set_low().unwrap();
                }
                1 => {
                    row_1.set_low().unwrap();
                }
                0 => {
                    row_0.set_low().unwrap();
                }
                _ => {}
            }
            for width in 0..COL_COUNT {
                let t = row * COL_COUNT + width;
                let element = unsafe { FRAME_BUFFER[t as usize] };

                led_clock.set_low().unwrap();
                if element.red {
                    led_data.set_high().unwrap();
                } else {
                    led_data.set_low().unwrap();
                }
                led_clock.set_high().unwrap();

                led_clock.set_low().unwrap();
                if element.blue {
                    led_data.set_high().unwrap();
                } else {
                    led_data.set_low().unwrap();
                }
                led_clock.set_high().unwrap();

                led_clock.set_low().unwrap();
                if element.green {
                    led_data.set_high().unwrap();
                } else {
                    led_data.set_low().unwrap();
                }
                led_clock.set_high().unwrap();
            }

            led_blank.set_low().unwrap();
            led_latch.set_high().unwrap();
            // Need a little delay here or the LEDs won't stay visible long enough
            delay.delay_us(1);
            led_latch.set_low().unwrap();
            led_blank.set_high().unwrap();
        }
    }
}

// End of file
