#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use miniz_oxide::inflate::decompress_slice_iter_to_slice;

static ZLIB_TEXT: &[u8; 42] = include_bytes!("../test_raw.gz");

/// Detect gzip header (if present) and returns the position where the deflate data starts.
/// Returns None if the input is not a valid gzip file.
/// Doesn't check or use CRC
fn find_gzip_header_end(data: &[u8]) -> Option<usize> {
    // Minimum gzip header is 10 bytes
    if data.len() < 10 {
        return None;
    }
    // Check magic number
    if data[0] != 0x1f || data[1] != 0x8b {
        return None;
    }
    // Check compression method is 8 (deflate). Wikipedia says this is always 8.
    if data[2] != 8 {
        return None;
    }
    // Parse the flags out of FLG
    // FLG.FHCRC (0x02): Header CRC16
    // FLG.FEXTRA (0x04): Extra field
    // FLG.FNAME (0x08): Original filename (null-terminated string)
    // FLG.FCOMMENT (0x10): Comment (null-terminated string)
    let flags = data[3];
    let header_crc16 = (flags & 0x02) != 0;
    let flg_fextra = (flags & 0x04) != 0;
    let flg_fname = (flags & 0x08) != 0;
    let flg_fcomment = (flags & 0x10) != 0;
    // We don't care about MTIME, XFL or OS. Move our pointer to XLEN (byte 10)
    let mut pos = 10;

    if flg_fextra {
        if pos + 2 > data.len() {
            return None;
        }
        let xlen = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
        pos += 2;
        if pos + xlen > data.len() {
            // If the extra length from xlen would make us parse past the end, abort
            return None;
        }
        pos += xlen;
    }

    /// Return the length of a null-terminated C string including the terminator
    fn cstring_len(data: &[u8]) -> Option<usize> {
        data.iter().position(|&b| b == 0).map(|pos| pos + 1)
    }

    if flg_fname {
        pos += cstring_len(&data[pos..])?;
    }

    if flg_fcomment {
        pos += cstring_len(&data[pos..])?;
    }

    if header_crc16 {
        if pos + 2 > data.len() {
            return None;
        }
        pos += 2;
    }

    Some(pos)
}

#[entry]
fn main() -> ! {
    info!("Program start");

    // test that miniz_oxide can decompress text from the gzip file
    if let Some(header_end) = find_gzip_header_end(ZLIB_TEXT) {
        let deflate_data = &ZLIB_TEXT[header_end..];
        let mut message_array = [0; 1024];
        let s = decompress_slice_iter_to_slice(
            &mut message_array,
            core::iter::once(deflate_data),
            false,
            false,
        )
        .unwrap();
        info!("Qty of decoded bytes: {}", s);
        let message = unsafe { str::from_utf8_unchecked(&message_array) };
        info!("Next line is decoded gz data\n{}", message);
    } else {
        info!("Not a valid gzip file");
    }

    info!("If we got here we didn't crash during decode - now we'll blink the LED to make it obvious without reading console that we're still running");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
