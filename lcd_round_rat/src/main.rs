//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use gc9a01a_driver::{Orientation, GC9A01A};

use rp2040_panic_usb_boot as _;

use ratatui::prelude::*;
use ratatui::widgets::Block;
use ratatui::widgets::Paragraph;
use ratatui::widgets::Wrap;
use ratatui::{Frame, Terminal};

use rp2040_hal::{
    self as hal,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
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

#[rp2040_hal::entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        // We need a pretty big heap for ratatui. if the device reconnects as UF2, you probably hit this limit
        const HEAP_SIZE: usize = 100000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    }

    // info!("Program start");
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
    timer.delay_ms(1000);

    // Turn the backlight on
    _lcd_bl.set_high().unwrap();

    let backend = EmbeddedBackend::new(&mut display, EmbeddedBackendConfig::default());
    let terminal = Terminal::new(backend);
    if let Ok(mut terminal) = terminal {
        // info!("terminal okay");
        let a = terminal.draw(draw);
        if a.is_ok() {
            // info!("draw success");
        } else {
            error_blink(&mut _lcd_bl, &mut timer.clone(), 200);
        }
    } else {
        error_blink(&mut _lcd_bl, &mut timer.clone(), 500);
    }

    loop {
        // TODO:  do drawing here.
        timer.delay_ms(1000);
    }
}

fn draw(frame: &mut Frame) {
    let text = "Ratatui on embedded devices!";
    let paragraph = Paragraph::new(text.dark_gray()).wrap(Wrap { trim: true });
    let bordered_block = Block::bordered()
        .border_style(Style::new().yellow())
        .title("Mousefood");
    frame.render_widget(paragraph.block(bordered_block), frame.area());
}
// End of file

fn error_blink(
    led: &mut impl embedded_hal::digital::OutputPin,
    timer: &mut impl embedded_hal::delay::DelayNs,
    delay: u32,
) {
    led.set_high().unwrap();
    timer.delay_ms(delay);
    led.set_low().unwrap();
    timer.delay_ms(delay);
}
