#![no_std]

use core::convert::TryInto;

use rp2040_hal::{
    gpio::{bank0::*, DynPin, FunctionConfig, FunctionPio0, Pin},
    pio::{
        PIOExt, PinDir, PinState, ShiftDirection, StateMachine, StateMachineIndex, Tx,
        UninitStateMachine, ValidStateMachine, PIO,
    },
};

pub struct DviPins {
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

pub struct DviPio<'pio, P, SM>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
{
    pio: &'pio mut PIO<P>,
    sm: StateMachine<SM, rp2040_hal::pio::Running>,
    tx: Tx<SM>,
    pins: DviPins,
}

impl<'pio, P, SM> DviPio<'pio, P, (P, SM)>
where
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    fn build_program() -> pio::Program<32_usize> {
        pio_proc::pio_asm!(
            ".side_set 2"
            ".origin 0"

            // Single-ended -> differential serial
            "out pc, 1    side 0b10"
            "out pc, 1    side 0b01"
        )
        .program
    }
}
