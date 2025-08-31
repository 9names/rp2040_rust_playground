//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use alloc::boxed::Box;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use ratatui::style::{Style, Stylize};
use ratatui::widgets::{Block, Paragraph, Wrap};
use rp235x_hal::clocks::init_clocks_and_plls;
use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::{self as hal, entry};
use rp235x_hal::{Clock, pac};

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::{DrawTarget, RgbColor},
};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use mipidsi::interface::SpiInterface;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::{models::ST7789, options::ColorInversion};
use panic_probe as _;

use ratatui::{Frame, Terminal};

extern crate alloc;
// Linked-List First Fit Heap allocator (feature = "llff")
use embedded_alloc::LlffHeap as Heap;
use mousefood::prelude::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 50000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    }

    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = cortex_m::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

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

    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico 2 board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico 2 W, the LED is not connected to any of the RP2350 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Setup SPI
    let display_dc = pins.gpio16.into_push_pull_output();
    let display_cs = pins.gpio17.into_push_pull_output();
    let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();

    let spi_bus = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi_bus = spi_bus.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        12_000_000u32.Hz(),
        embedded_hal::spi::MODE_3,
    );
    let excl_spi_dev = ExclusiveDevice::new(spi_bus, display_cs, timer).unwrap();
    let mut rst_pin = pins.gpio22.into_push_pull_output();
    rst_pin.set_low().unwrap();

    let buffer: alloc::boxed::Box<[u8; 512]> = alloc::boxed::Box::new([0_u8; 512]);

    //let di = SpiInterface::new(excl_spi_dev, display_dc, &mut buffer);
    let di = SpiInterface::new(excl_spi_dev, display_dc, Box::leak(buffer));
    let orientation = Orientation::new().rotate(Rotation::Deg90);

    let mut display = mipidsi::Builder::new(ST7789, di)
        .display_size(135u16, 240u16)
        .display_offset(52, 40)
        .invert_colors(ColorInversion::Inverted)
        .reset_pin(rst_pin)
        .orientation(orientation)
        .init(&mut timer)
        .unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    // let mut buffer: [u8; 512] = [0_u8; 512];
    let backend = EmbeddedBackend::new(&mut display, EmbeddedBackendConfig::default());
    let terminal = Terminal::new(backend);
    if let Ok(mut terminal) = terminal {
        info!("terminal okay");
        let a = terminal.draw(draw);
        if a.is_ok() {
            info!("draw success");
        } else {
            info!("draw fail")
        }
    } else {
        info!("terminal not okay");
    }
    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        timer.delay_ns(500_000_000);
        info!("off!");
        led_pin.set_low().unwrap();
        timer.delay_ns(500_000_000);
    }
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 Template"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

fn draw(frame: &mut Frame) {
    let text = "Ratatui on embedded devices!";
    let paragraph = Paragraph::new(text.dark_gray()).wrap(Wrap { trim: true });
    let bordered_block = Block::bordered()
        .border_style(Style::new().yellow())
        .title("Mousefood");
    frame.render_widget(paragraph.block(bordered_block), frame.area());
}

// End of file
