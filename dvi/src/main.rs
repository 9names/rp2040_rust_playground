//! DVI demo
//!
//!   This demo runs entirely from RAM for performance reasons

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_halt as _;

use rp2040_hal::{
    self as hal,
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    watchdog::Watchdog,
};

use dvipio::*;

// This boot2 will copy the entire program to RAM before running it
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;

pub const FRAME_WIDTH: usize = 320;
pub const FRAME_HEIGHT: usize = 240;
pub static mut FRAMEBUF: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0; FRAME_WIDTH * FRAME_HEIGHT];

#[entry]
fn main() -> ! {
    unsafe {
        // Release spinlocks since they are not freed on soft reset
        hal::sio::spinlock_reset();
    }
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on every board that supports USB boot is 12Mhz
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

    // All the regular pins on the board
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    loop {
        led_pin.toggle().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
