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
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use bsp::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{DynPin, FunctionPio0, Pin},
        pac,
        pio::{
            PIOExt, PinDir, PinState, Rx, ShiftDirection, StateMachine, StateMachineIndex, Tx,
            UninitStateMachine, ValidStateMachine, PIO,
        },
        sio::Sio,
        watchdog::Watchdog,
        Timer,
    },
    Gp8Pio0,
};
use pio::{Instruction, InstructionOperands, SideSet};
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

const ROW_COUNT2: u32 = 7;
const ROW_BYTES: u32 = 12;
const BCD_FRAMES: u32 = 15; // includes fet discharge frame
const BITSTREAM_LENGTH: u32 = (ROW_COUNT2 * ROW_BYTES * BCD_FRAMES);
const WIDTH: usize = 16;
const HEIGHT: usize = 7;

// must be aligned for 32bit dma transfer
static mut BITSTREAM: [u8; BITSTREAM_LENGTH as usize] = [0; BITSTREAM_LENGTH as usize];

const GAMMA_14BIT: [u16; 256] = [
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

fn init_bitstream() {
    // initialise the bcd timing values and row selects in the bitstream
    for row in 0..HEIGHT {
        for frame in 0..BCD_FRAMES {
            // determine offset in the buffer for this row/frame
            let offset = (row * ROW_BYTES as usize * BCD_FRAMES as usize)
                + (ROW_BYTES as usize * frame as usize);

            let row_select_offset = offset + 9;
            let bcd_offset = offset + 10;

            // the last bcd frame is used to allow the fets to discharge to avoid ghosting
            if (frame == BCD_FRAMES - 1u32) {
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

fn set_pixel(x: usize, y: usize, r: usize, g: usize, b: usize) {
    if x >= WIDTH || y >= HEIGHT {
        return;
    }

    // make those coordinates sane
    let mut x = (WIDTH - 1) - x;

    // work out the byte offset of this pixel
    let byte_offset = x / 2;

    // check if it's the high or low nibble and create mask and shift value
    let shift = if x % 2 == 0 { 0 } else { 4 };
    let nibble_mask = 0b00001111 << shift;

    let mut gr = GAMMA_14BIT[r];
    let mut gg = GAMMA_14BIT[g];
    let mut gb = GAMMA_14BIT[b];

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    //let pio = pac.PIO0;
    // prepare the PIO program
    //let mut a = pio::Assembler::<32>::new_with_side_set(SIDESET);

    /*
    ; set pins:
    ; 0: data (base)
    ; 1: clock
    ; 2: latch
    ; 3: blank

    ; sideset pin: clock

    ; out pins:
    ; 0: row 6 select
    ; 1: row 5 select
    ; 2: row 4 select
    ; 3: row 3 select
    ; 4: row 2 select
    ; 5: row 1 select
    ; 6: row 0 select
    */

    let mut program = pio_proc::pio_asm!(
        ".side_set 1 opt"

        ".wrap_target"
        // clock out 16 pixels worth of data
            "set y, 15"                // 15 because `jmp` test is pre decrement

            "pixels:"

            "pull ifempty"

            // dummy bit used to align pixel data to nibbles
            "out null, 1  "            // discard

            // red bit
            "out x, 1       side 0 "   // pull in first bit from OSR into register X, clear clock
            "set pins, 8"              // clear data bit (maintain blank)
            "jmp !x endr"              // if bit was zero jump to endr
            "set pins, 9"              // set data bit (maintain blank)
        "endr: "                       //
            "nop            side 1"    // clock in bit

            // green bit
            "out x, 1       side 0"    // pull in first bit from OSR into register X, clear clock
            "set pins, 8"              // clear data bit (maintain blank)
            "jmp !x endg"              // if bit was zero jump to endg
            "set pins, 9"              // set data bit (maintain blank)
        "endg:"                        //
            "nop            side 1"    // clock in bit

            // blue bit
            "out x, 1       side 0"    // pull in first bit from OSR into register X, clear clock
            "set pins, 8"              // clear data bit (maintain blank)
            "jmp !x endb"              // if bit was zero jump to endb
            "set pins, 9"              // set data bit (maintain blank)
        "endb:"                        //
            "nop            side 1"    // clock in bit

            "jmp y-- pixels"           // jump back to start of pixel loop

            "pull"

        // dummy byte to 32 bit align row data
            "out null, 8"

        // select active row
            "out null, 1"            // discard dummy bit
            "out pins, 7"              // output row selection mask

        // pull bcd tick count into x register
            "out x, 16"

        // set latch pin to output column data on shift registers
            "set pins, 12"             // set latch pin (while keeping blank high)

        // set blank pin to enable column drivers
            "set pins, 4"

        "bcd_count:"
            "jmp x-- bcd_count "      // loop until bcd delay complete

        // disable all row outputs
            "set x, 0"                // load x register with 0 (we can't set more than 5 bits at a time)
            "mov pins, !x"            // write inverted x (0xff) to row pins latching them all high

        // disable led output (blank) and clear latch pin
            "set pins, 8"

        ".wrap"
    )
    .program;

    let mut led_data: Pin<_, FunctionPio0> = pins.gpio8.into_mode();
    let mut led_clock: Pin<_, FunctionPio0> = pins.gpio9.into_mode();
    let mut led_latch: Pin<_, FunctionPio0> = pins.gpio10.into_mode();
    let mut led_blank: Pin<_, FunctionPio0> = pins.gpio11.into_mode();
    let mut row_0: Pin<_, FunctionPio0> = pins.gpio22.into_mode();
    let mut row_1: Pin<_, FunctionPio0> = pins.gpio21.into_mode();
    let mut row_2: Pin<_, FunctionPio0> = pins.gpio20.into_mode();
    let mut row_3: Pin<_, FunctionPio0> = pins.gpio19.into_mode();
    let mut row_4: Pin<_, FunctionPio0> = pins.gpio18.into_mode();
    let mut row_5: Pin<_, FunctionPio0> = pins.gpio17.into_mode();
    let mut row_6: Pin<_, FunctionPio0> = pins.gpio16.into_mode();

    let mut led_data: DynPin = led_data.into();
    let mut led_clock: DynPin = led_clock.into();
    let mut led_latch: DynPin = led_latch.into();
    let mut led_blank: DynPin = led_blank.into();
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

    // Install the program into PIO instruction memory.
    let installed = pio.install(&program).unwrap();
    let (mut sm, rx, mut tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
        // use both RX & TX FIFO
        .buffers(bsp::hal::pio::Buffers::OnlyTx)
        .set_pins(led_data.id().num, 4)
        .out_pins(row_6.id().num, 7)
        .side_set_pin_base(9)
        .clock_divisor(100.0) // as slow as possible
        .build(sm0);

    sm.set_pins([
        (led_data.id().num, PinState::Low),
        (led_clock.id().num, PinState::Low),
        (led_latch.id().num, PinState::Low),
        (led_blank.id().num, PinState::Low),
        (row_0.id().num, PinState::Low),
        (row_1.id().num, PinState::Low),
        (row_2.id().num, PinState::Low),
        (row_3.id().num, PinState::Low),
        (row_4.id().num, PinState::Low),
        (row_5.id().num, PinState::Low),
        (row_6.id().num, PinState::Low),
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

    let mut x = 0;
    let mut y = 0;

    // for y in 0..HEIGHT - 1 {
    //     for x in 0..WIDTH - 1 {
    //         set_pixel(x, y, 0, 0, 255);
    //     }
    // }

    loop {
        set_pixel(x, y, 0, 0, 255);
        x += 1;

        if x > WIDTH {
            x = 0;
            y += 1;
        }
        if y > HEIGHT {
            y = 0;
        }
        set_pixel(x, y, 0, 255, 0);
        let _ = led_pin.toggle();
        for i in 0..BITSTREAM_LENGTH as usize / 4 {
            let is = i * 4;
            let bitval: u32 = unsafe {
                (BITSTREAM[is] as u32) << 24
                    | (BITSTREAM[is + 1] as u32) << 16
                    | (BITSTREAM[is + 2] as u32) << 8
                    | (BITSTREAM[is + 3] as u32)
            };
            // wait for space
            while tx.is_full() {}
            tx.write(bitval);
        }
    }

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
