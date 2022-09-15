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

use core::convert::TryInto;

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;

use bsp::hal::{
    clocks::init_clocks_and_plls,
    gpio::{DynPin, FunctionPio0, Pin},
    pac,
    pio::{PIOExt, PinDir, PinState, ShiftDirection, UninitStateMachine, ValidStateMachine, PIO},
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico as bsp;

extern "C" {
    //pub fn init_bitstream();
    //pub fn set_pixel(x: u8, y: u8, r: u8, g: u8, b: u8);
    // pub static mut BITSTREAM: [u8; 1260usize];

}

const ROW_COUNT2: usize = 7;
const ROW_BYTES: usize = 12;
const BCD_FRAMES: usize = 15; // includes fet discharge frame
const BITSTREAM_LENGTH: usize = (ROW_COUNT2 * ROW_BYTES * BCD_FRAMES);
const WIDTH: usize = 16;
const HEIGHT: usize = 7;

// must be aligned for 32bit dma transfer
#[no_mangle]
pub static mut BITSTREAM: [u8; BITSTREAM_LENGTH as usize] = [0; BITSTREAM_LENGTH as usize];

static GAMMA_14BIT: [u16; 256] = [
    0, 0, 0, 1, 2, 3, 4, 6, 8, 10, 13, 16, 20, 23, 28, 32, 37, 42, 48, 54, 61, 67, 75, 82, 90, 99,
    108, 117, 127, 137, 148, 159, 170, 182, 195, 207, 221, 234, 249, 263, 278, 294, 310, 326, 343,
    361, 379, 397, 416, 435, 455, 475, 496, 517, 539, 561, 583, 607, 630, 654, 679, 704, 730, 756,
    783, 810, 838, 866, 894, 924, 953, 983, 1014, 1045, 1077, 1110, 1142, 1176, 1210, 1244, 1279,
    1314, 1350, 1387, 1424, 1461, 1499, 1538, 1577, 1617, 1657, 1698, 1739, 1781, 1823, 1866, 1910,
    1954, 1998, 2044, 2089, 2136, 2182, 2230, 2278, 2326, 2375, 2425, 2475, 2525, 2577, 2629, 2681,
    2734, 2787, 2841, 2896, 2951, 3007, 3063, 3120, 3178, 3236, 3295, 3354, 3414, 3474, 3535, 3596,
    3658, 3721, 3784, 3848, 3913, 3978, 4043, 4110, 4176, 4244, 4312, 4380, 4449, 4519, 4589, 4660,
    4732, 4804, 4876, 4950, 5024, 5098, 5173, 5249, 5325, 5402, 5479, 5557, 5636, 5715, 5795, 5876,
    5957, 6039, 6121, 6204, 6287, 6372, 6456, 6542, 6628, 6714, 6801, 6889, 6978, 7067, 7156, 7247,
    7337, 7429, 7521, 7614, 7707, 7801, 7896, 7991, 8087, 8183, 8281, 8378, 8477, 8576, 8675, 8775,
    8876, 8978, 9080, 9183, 9286, 9390, 9495, 9600, 9706, 9812, 9920, 10027, 10136, 10245, 10355,
    10465, 10576, 10688, 10800, 10913, 11027, 11141, 11256, 11371, 11487, 11604, 11721, 11840,
    11958, 12078, 12198, 12318, 12440, 12562, 12684, 12807, 12931, 13056, 13181, 13307, 13433,
    13561, 13688, 13817, 13946, 14076, 14206, 14337, 14469, 14602, 14735, 14868, 15003, 15138,
    15273, 15410, 15547, 15685, 15823, 15962, 16102, 16242, 16383,
];

fn init_bitstream_rs() {
    // initialise the bcd timing values and row selects in the bitstream
    for row in 0..HEIGHT {
        for frame in 0..BCD_FRAMES {
            // determine offset in the buffer for this row/frame
            let offset = (row * ROW_BYTES as usize * BCD_FRAMES as usize)
                + (ROW_BYTES as usize * frame as usize);

            let row_select_offset = offset + 9;
            let bcd_offset = offset + 10;

            // the last bcd frame is used to allow the fets to discharge to avoid ghosting
            if (frame == BCD_FRAMES - 1usize) {
                let bcd_ticks: u16 = 65535;
                unsafe {
                    BITSTREAM[row_select_offset] = 0b11111111;
                    BITSTREAM[bcd_offset + 1] = ((bcd_ticks & 0xff00) >> 8) as u8;
                    BITSTREAM[bcd_offset] = (bcd_ticks & 0xff) as u8;
                    for col in 0..6 {
                        BITSTREAM[offset + col] = 0xff;
                    }
                }
            } else {
                let row_select_mask = !(1 << (7 - row));
                let bcd_ticks: u16 = 1 << frame;
                unsafe {
                    BITSTREAM[row_select_offset] = row_select_mask;
                    BITSTREAM[bcd_offset + 1] = ((bcd_ticks & 0xff00) >> 8) as u8;
                    BITSTREAM[bcd_offset] = (bcd_ticks & 0xff) as u8;
                }
            }
        }
    }

    // unsafe {

    // }
}

fn set_pixel(x: u8, y: u8, r: u8, g: u8, b: u8) {
    // unsafe {
    //     set_pixel(x, y, r, g, b);
    // }
    // return;
    let x = x as usize;
    let y = y as usize;
    if x >= WIDTH || y >= HEIGHT {
        return;
    }

    // make those coordinates sane
    let x = (WIDTH - 1) - x;

    // work out the byte offset of this pixel
    let byte_offset = x / 2;

    // check if it's the high or low nibble and create mask and shift value
    let shift = if x % 2 == 0 { 0 } else { 4 };
    let nibble_mask = 0b00001111 << shift;

    let mut gr = GAMMA_14BIT[r as usize];
    let mut gg = GAMMA_14BIT[g as usize];
    let mut gb = GAMMA_14BIT[b as usize];

    // set the appropriate bits in the separate bcd frames
    for frame in 0..BCD_FRAMES {
        // determine offset in the buffer for this row/frame
        let offset =
            (y * ROW_BYTES as usize * BCD_FRAMES as usize) + (ROW_BYTES as usize * frame as usize);

        let mut rgbd = ((gr & 0b1) << 1) | ((gg & 0b1) << 3) | ((gb & 0b1) << 2);

        // shift to correct nibble
        rgbd <<= shift;

        // clear existing data
        unsafe {
            BITSTREAM[offset + byte_offset] &= !nibble_mask;
        }

        // set new data
        unsafe {
            BITSTREAM[offset + byte_offset] |= rgbd as u8;
        }

        gr >>= 1;
        gg >>= 1;
        gb >>= 1;
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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let program = pio_proc::pio_file!("./src/pico_unicorn.pio");

    let mut led_blank: Pin<_, FunctionPio0> = pins.gpio11.into_mode();
    let mut led_latch: Pin<_, FunctionPio0> = pins.gpio10.into_mode();
    let mut led_clock: Pin<_, FunctionPio0> = pins.gpio9.into_mode();
    let mut led_data: Pin<_, FunctionPio0> = pins.gpio8.into_mode();
    let mut row_0: Pin<_, FunctionPio0> = pins.gpio22.into_mode();
    let mut row_1: Pin<_, FunctionPio0> = pins.gpio21.into_mode();
    let mut row_2: Pin<_, FunctionPio0> = pins.gpio20.into_mode();
    let mut row_3: Pin<_, FunctionPio0> = pins.gpio19.into_mode();
    let mut row_4: Pin<_, FunctionPio0> = pins.gpio18.into_mode();
    let mut row_5: Pin<_, FunctionPio0> = pins.gpio17.into_mode();
    let mut row_6: Pin<_, FunctionPio0> = pins.gpio16.into_mode();

    let mut led_blank: DynPin = led_blank.into();
    let mut led_latch: DynPin = led_latch.into();
    let mut led_clock: DynPin = led_clock.into();
    let mut led_data: DynPin = led_data.into();
    let mut row_0: DynPin = row_0.into();
    let mut row_1: DynPin = row_1.into();
    let mut row_2: DynPin = row_2.into();
    let mut row_3: DynPin = row_3.into();
    let mut row_4: DynPin = row_4.into();
    let mut row_5: DynPin = row_5.into();
    let mut row_6: DynPin = row_6.into();

    // Buttons
    let mut btn_a = pins.gpio12.into_floating_input();
    let mut btn_b = pins.gpio13.into_floating_input();
    let mut btn_x = pins.gpio14.into_floating_input();
    let mut btn_y = pins.gpio15.into_floating_input();

    let loop_update_freq = 1000;
    let mut loop_counter = 0;
    init_bitstream_rs();
    // Install the program into PIO instruction memory.
    let installed = pio.install(&program.program).unwrap();

    let (mut sm, rx, mut tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
        .buffers(bsp::hal::pio::Buffers::OnlyTx)
        .out_pins(row_6.id().num, 7)
        .side_set_pin_base(led_clock.id().num)
        .set_pins(led_data.id().num, 4)
        .autopull(true)
        .pull_threshold(32)
        .out_shift_direction(ShiftDirection::Right)
        .clock_divisor(1.0)
        .build(sm0);

    sm.set_pins([
        (led_data.id().num, PinState::High),
        (led_clock.id().num, PinState::High),
        (led_latch.id().num, PinState::High),
        (led_blank.id().num, PinState::High),
        (row_0.id().num, PinState::High),
        (row_1.id().num, PinState::High),
        (row_2.id().num, PinState::High),
        (row_3.id().num, PinState::High),
        (row_4.id().num, PinState::High),
        (row_5.id().num, PinState::High),
        (row_6.id().num, PinState::High),
    ]);
    sm.set_pindirs([
        (led_data.id().num, PinDir::Output),
        (led_clock.id().num, PinDir::Output),
        (led_latch.id().num, PinDir::Output),
        (led_blank.id().num, PinDir::Output),
        (row_0.id().num, PinDir::Output),
        (row_1.id().num, PinDir::Output),
        (row_2.id().num, PinDir::Output),
        (row_3.id().num, PinDir::Output),
        (row_4.id().num, PinDir::Output),
        (row_5.id().num, PinDir::Output),
        (row_6.id().num, PinDir::Output),
    ]);
    let _sm = sm.start();
    let mut led_pin = pins.led.into_push_pull_output();

    let _ = led_pin.toggle();
    loop {
        let colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255)];
        for color in colours {
            for y in 0..HEIGHT as u8 {
                for x in 0..WIDTH as u8 {
                    set_pixel(x, y, color.0, color.1, color.2);
                    for batch in unsafe { BITSTREAM.chunks_exact(4) } {
                        while !tx.write(u32::from_le_bytes(unsafe {
                            batch.try_into().unwrap_unchecked()
                        })) {}
                    }
                    set_pixel(x, y, 0, 0, 0);
                }
            }
        }
    }
}

// End of file
