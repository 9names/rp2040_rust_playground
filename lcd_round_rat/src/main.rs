//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use gc9a01a_driver::{Orientation, GC9A01A};
use panic_probe as _;

use rp2040_hal::{
    self as hal,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
};
use embedded_hal::delay::DelayNs;

const LCD_WIDTH: u32 = 240;
const LCD_HEIGHT: u32 = 240;

extern crate alloc;
// Linked-List First Fit Heap allocator (feature = "llff")
use embedded_alloc::LlffHeap as Heap;
use mousefood::prelude::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    }

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

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Initialize LCD pins
    let lcd_dc = pins.gpio8.into_push_pull_output();
    let lcd_cs = pins.gpio9.into_push_pull_output();
    let lcd_clk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
    let lcd_mosi = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
    let lcd_rst = pins
        .gpio13
        .into_push_pull_output_in_state(hal::gpio::PinState::High);
    let mut _lcd_bl = pins
        .gpio25
        .into_push_pull_output_in_state(hal::gpio::PinState::Low);

    // Initialize SPI
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_mosi, lcd_clk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        40.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // Initialize the display
    let mut display = GC9A01A::new(spi, lcd_dc, lcd_cs, lcd_rst, false, LCD_WIDTH, LCD_HEIGHT);
    display.init(&mut timer).unwrap();
    display.set_orientation(&Orientation::Landscape).unwrap();

    // Clear the screen before turning on the backlight
    display.clear(Rgb565::BLACK).unwrap();
    // _lcd_bl.into_push_pull_output_in_state(hal::gpio::PinState::High);
    timer.delay_ms(1000);
    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    // let mut led_pin = pins.gpio25.into_push_pull_output();
    let backend = EmbeddedBackend::new(&mut display, EmbeddedBackendConfig::default());
    // let mut terminal = Terminal::new(backend)?;

    loop {
        info!("on!");
        _lcd_bl.set_high().unwrap();
        timer.delay_ms(500);
        info!("off!");
        _lcd_bl.set_low().unwrap();
        timer.delay_ms(500);
    }
}

// End of file
