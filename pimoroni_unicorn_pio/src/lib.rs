#![no_std]

use core::convert::TryInto;

use bsp::hal::{
    gpio::{bank0::*, DynPin, FunctionConfig, FunctionPio0, Pin},
    pio::{
        PIOExt, PinDir, PinState, ShiftDirection, StateMachine, StateMachineIndex, Tx,
        UninitStateMachine, ValidStateMachine, PIO,
    },
};
use rp_pico as bsp;

const ROW_COUNT2: usize = 7;
const ROW_BYTES: usize = 12;
const BCD_FRAMES: usize = 15; // includes fet discharge frame
const BITSTREAM_LENGTH: usize = ROW_COUNT2 * ROW_BYTES * BCD_FRAMES;
pub const WIDTH: usize = 16;
pub const HEIGHT: usize = 7;

// TODO: must be aligned for 32bit dma transfer
#[repr(C, align(4))]
struct Bitstream {
    data: [u8; BITSTREAM_LENGTH as usize],
}

impl Bitstream {
    const fn new() -> Self {
        Self {
            data: [0; BITSTREAM_LENGTH as usize],
        }
    }
}

#[no_mangle]
static mut BITSTREAM: Bitstream = Bitstream::new();

pub struct UnicornPins {
    pub led_blank: Pin<Gpio11, FunctionPio0>,
    pub led_latch: Pin<Gpio10, FunctionPio0>,
    pub led_clock: Pin<Gpio9, FunctionPio0>,
    pub led_data: Pin<Gpio8, FunctionPio0>,
    pub row_0: Pin<Gpio22, FunctionPio0>,
    pub row_1: Pin<Gpio21, FunctionPio0>,
    pub row_2: Pin<Gpio20, FunctionPio0>,
    pub row_3: Pin<Gpio19, FunctionPio0>,
    pub row_4: Pin<Gpio18, FunctionPio0>,
    pub row_5: Pin<Gpio17, FunctionPio0>,
    pub row_6: Pin<Gpio16, FunctionPio0>,
}

pub struct UnicornDynPins([DynPin; 11]);

/// Instance of Pico Unicorn display
pub struct Unicorn<'pio, P, SM>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
{
    #[allow(dead_code)]
    pio: &'pio mut PIO<P>,
    #[allow(dead_code)]
    sm: StateMachine<SM, rp_pico::hal::pio::Running>,
    tx: Tx<SM>,
    #[allow(dead_code)]
    pins: UnicornDynPins,
}

impl<'pio, P, SM> Unicorn<'pio, P, (P, SM)>
where
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    /// Creates a new instance of this driver.
    ///
    /// Note: the PIO must have been reset before using this driver.
    pub fn new(
        pio: &'pio mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        pins: UnicornPins,
    ) -> Unicorn<'pio, P, (P, SM)> {
        let program = Self::build_program();
        Self::init_bitstream_rs();
        // Install the program into PIO instruction memory.
        let installed = pio.install(&program).unwrap();

        let led_blank: DynPin = pins.led_blank.into();
        let led_latch: DynPin = pins.led_latch.into();
        let led_clock: DynPin = pins.led_clock.into();
        let led_data: DynPin = pins.led_data.into();
        let row_0: DynPin = pins.row_0.into();
        let row_1: DynPin = pins.row_1.into();
        let row_2: DynPin = pins.row_2.into();
        let row_3: DynPin = pins.row_3.into();
        let row_4: DynPin = pins.row_4.into();
        let row_5: DynPin = pins.row_5.into();
        let row_6: DynPin = pins.row_6.into();

        let (mut sm, _rx, tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
            .buffers(bsp::hal::pio::Buffers::OnlyTx)
            .out_pins(row_6.id().num, 7)
            .side_set_pin_base(led_clock.id().num)
            .set_pins(led_data.id().num, 4)
            .autopull(true)
            .pull_threshold(32)
            .out_shift_direction(ShiftDirection::Right)
            .clock_divisor_fixed_point(1, 0)
            .build(sm);

        sm.set_pins([
            (led_blank.id().num, PinState::High),
            (led_latch.id().num, PinState::High),
            (led_clock.id().num, PinState::High),
            (led_data.id().num, PinState::High),
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
        let sm = sm.start();

        let pins: UnicornDynPins = UnicornDynPins([
            led_blank, led_latch, led_clock, led_data, row_0, row_1, row_2, row_3, row_4, row_5,
            row_6,
        ]);
        Self { pio, sm, tx, pins }
    }

    fn build_program() -> pio::Program<32_usize> {
        pio_proc::pio_asm!(
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
                "out null, 1"              // discard dummy bit
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
        .program
    }

    pub fn init_bitstream_rs() {
        // initialise the bcd timing values and row selects in the bitstream
        for row in 0..HEIGHT {
            for frame in 0..BCD_FRAMES {
                // determine offset in the buffer for this row/frame
                let offset = (row * ROW_BYTES as usize * BCD_FRAMES as usize)
                    + (ROW_BYTES as usize * frame as usize);

                let row_select_offset = offset + 9;
                let bcd_offset = offset + 10;

                // the last bcd frame is used to allow the fets to discharge to avoid ghosting
                if frame == BCD_FRAMES - 1usize {
                    let bcd_ticks: u16 = 65535;
                    unsafe {
                        BITSTREAM.data[row_select_offset] = 0b11111111;
                        BITSTREAM.data[bcd_offset + 1] = ((bcd_ticks & 0xff00) >> 8) as u8;
                        BITSTREAM.data[bcd_offset] = (bcd_ticks & 0xff) as u8;
                        for col in 0..6 {
                            BITSTREAM.data[offset + col] = 0xff;
                        }
                    }
                } else {
                    let row_select_mask = !(1 << (7 - row));
                    let bcd_ticks: u16 = 1 << frame;
                    unsafe {
                        BITSTREAM.data[row_select_offset] = row_select_mask;
                        BITSTREAM.data[bcd_offset + 1] = ((bcd_ticks & 0xff00) >> 8) as u8;
                        BITSTREAM.data[bcd_offset] = (bcd_ticks & 0xff) as u8;
                    }
                }
            }
        }
    }
}

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

impl<'pio, P, SM> Unicorn<'pio, P, SM>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
{
    #[inline(always)]
    pub fn set_pixel(&mut self, coord: Point, color: Rgb888) {
        self.set_pixel_rgb(
            coord.x as u8,
            coord.y as u8,
            color.r(),
            color.g(),
            color.b(),
        )
    }

    pub fn set_pixel_rgb(&mut self, x: u8, y: u8, r: u8, g: u8, b: u8) {
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
            let offset = (y * ROW_BYTES as usize * BCD_FRAMES as usize)
                + (ROW_BYTES as usize * frame as usize);

            let mut rgbd = ((gr & 0b1) << 1) | ((gg & 0b1) << 3) | ((gb & 0b1) << 2);

            // shift to correct nibble
            rgbd <<= shift;

            // clear existing data
            unsafe {
                BITSTREAM.data[offset + byte_offset] &= !nibble_mask;
            }

            // set new data
            unsafe {
                BITSTREAM.data[offset + byte_offset] |= rgbd as u8;
            }

            gr >>= 1;
            gg >>= 1;
            gb >>= 1;
        }
    }

    pub fn draw(&mut self) {
        for batch in unsafe { BITSTREAM.data.chunks_exact(4) } {
            while !self.tx.write(u32::from_le_bytes(unsafe {
                batch.try_into().unwrap_unchecked()
            })) {}
        }
    }
}

#[cfg(feature = "graphics")]
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::Size,
    geometry::{Dimensions, OriginDimensions},
    pixelcolor::Rgb888,
    prelude::*,
};

#[cfg(feature = "graphics")]
impl<'pio, P, SM> DrawTarget for Unicorn<'pio, P, SM>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
{
    type Color = Rgb888;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let bb = self.bounding_box();
        pixels
            .into_iter()
            .filter(|Pixel(pos, _color)| bb.contains(*pos))
            .for_each(|Pixel(pos, color)| self.set_pixel(pos, color));
        Ok(())
    }
}

impl<'pio, P, SM> OriginDimensions for Unicorn<'pio, P, SM>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
