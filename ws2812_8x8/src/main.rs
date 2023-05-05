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

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Timer,
};
// Expose our board support package as `bsp` to make porting code between boards easier
use rp_pico as bsp;

// These dependencies are needed for driving ws2812
use hal::pio::PIOExt;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

/// This is the number of ws2812 LEDs we are driving
const STRIP_LEN: usize = 64;

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

    // Split up PIO state machine 0 so that Ws2812 can use it
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // Create a count down timer for the Ws2812 instance
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    // Instantiate the Ws2812 driver
    let mut ws = Ws2812::new(
        // Use neopixel pin (19) on the Adafruit macropad (which is GPIO19 of the rp2040 chip)
        // for the LED data output:
        pins.gpio22.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // A buffer for storing the colour of each RGB LED
    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    // A counter we increment each pass through the main loop that we
    // can pass to an algorithm to get a different state each time
    let mut t = 0.0;

    // Bring down the overall brightness of the LEDS
    // so you don't hurt your eyes.
    //
    // Brightness also affects power consumption
    // Measurements from my PinePower (low accuracy)
    // 64 LEDs running the RGB demo:
    // - strip_brightness 255: 1A
    // - strip_brightness 128: 0.6A
    // - strip_brightness 64: 0.3A
    // 64 LEDS with Red, Green and Blue set to the same brightness (white)
    // - 255: 1.6A
    // - 128: 1A
    // - 64: 0.1A
    let strip_brightness = 128u8; // Limit brightness to 64/256

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;
    // Import the `sin` function for a smooth hue animation from the rp2040 ROM:
    let sin = hal::rom_data::float_funcs::fsin::ptr();

    loop {
        for (i, led) in leds.iter_mut().enumerate() {
            // An offset for each LED so they aren't all the same colour
            let hue_offs = i as f32 / 5.0;

            let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
            // Bring -1..1 sine range to 0..1 range:
            let sin_01 = (sin_11 + 1.0) * 0.5;

            let hue = 360.0 * sin_01;
            let sat = 1.0;
            let val = 1.0;

            let rgb = hsv2rgb_u8(hue, sat, val);
            *led = rgb.into();
        }

        // Write the values from the leds array to the ws2812 chain
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        // Wait a bit until calculating the next frame:
        delay.delay_ms(16); // ~60 FPS

        // Increase the time counter variable
        t += (16.0 / 1000.0) * animation_speed;
        // ensure that it stay between 0.0 and 1.0
        while t > 1.0 {
            t -= 1.0;
            // Toggle the Pico's led so we can show the code is working,
            // even if the ws2812 is not wired up correctly
            // This should happen once every 12 seconds
            led_pin.toggle().unwrap();
        }
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}

// End of file
