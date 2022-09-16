//! Quick port of my plasma code for the unicorn PIO library

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics_core::pixelcolor::Rgb888;
use embedded_hal::digital::v2::{InputPin, ToggleableOutputPin};
use panic_probe as _;
use pimoroni_unicorn_pio::UnicornPins;

use bsp::hal::{
    clocks::init_clocks_and_plls, gpio::DynPin, pac, pio::PIOExt, sio::Sio, watchdog::Watchdog,
};
use rp_pico as bsp;

use embedded_graphics::prelude::*;

struct Buttons {
    a: DynPin,
    b: DynPin,
    x: DynPin,
    y: DynPin,
}

impl Buttons {
    fn new(a: DynPin, b: DynPin, x: DynPin, y: DynPin) -> Self {
        Buttons { a, b, x, y }
    }

    pub fn read(&self) -> ButtonState {
        ButtonState::new(
            self.a.is_low().unwrap(),
            self.b.is_low().unwrap(),
            self.x.is_low().unwrap(),
            self.y.is_low().unwrap(),
        )
    }
}

#[derive(Copy, Clone, Debug)]
struct ButtonState {
    a: bool,
    b: bool,
    x: bool,
    y: bool,
}

impl ButtonState {
    fn new(a: bool, b: bool, x: bool, y: bool) -> Self {
        Self { a, b, x, y }
    }

    /// if a new button pressed, and that press is true
    fn diff(&self, newstate: ButtonState) -> ButtonState {
        ButtonState::new(
            self.a != newstate.a && newstate.a,
            self.b != newstate.b && newstate.b,
            self.x != newstate.x && newstate.x,
            self.y != newstate.y && newstate.y,
        )
    }

    fn and(&self, mask: ButtonState) -> ButtonState {
        ButtonState::new(
            self.a == mask.a,
            self.b == mask.b,
            self.x == mask.x,
            self.y == mask.y,
        )
    }

    fn pressed(&self) -> ButtonState {
        self.and(ButtonState::new(true, true, true, true))
    }
}

struct ButtonHandler {
    buttons: Buttons,
    last: ButtonState,
}
impl ButtonHandler {
    fn new(a: DynPin, b: DynPin, x: DynPin, y: DynPin) -> Self {
        let buttons = Buttons::new(a, b, x, y);

        let last = buttons.read();
        ButtonHandler { buttons, last }
    }
    fn read(&mut self) -> ButtonState {
        let new_buttons = self.buttons.read();
        let pressed = self.last.diff(new_buttons).pressed();
        self.last = new_buttons;
        pressed
    }
}

struct F32Math {
    sin: extern "C" fn(f32) -> f32,
    sqrt: extern "C" fn(f32) -> f32,
}

impl F32Math {
    /// Formula for plasma at any particular address
    fn plasma_pixel(&self, x: f32, y: f32) -> f32 {
        let sin = self.sin;
        let sqrt = self.sqrt;
        (sin(x / 16.0) + sin(y / 8.0) + sin((x + y) / 16.0) + sin(sqrt(x * x + y * y) / 8.0) + 4.0)
            / 8.0
    }
}

fn hsv_to_rgb(hue: f32, saturation: f32, value: f32) -> Rgb888 {
    use micromath::F32Ext as _; // needed for rem_euclid, floor, abs and round

    let c = value * saturation;
    let h = hue * 6.0;
    let j = h.rem_euclid(2.0);
    let k = h.rem_euclid(6.0);

    let x = c * (1.0 - (j - 1.0).abs());
    let m = value - c;
    let (red, green, blue) = match k.floor() as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    // Convert back to RGB888
    let (ri, gi, bi) = (
        ((red + m) * 255.0f32).round() as u8,
        ((green + m) * 255.0f32).round() as u8,
        ((blue + m) * 255.0f32).round() as u8,
    );
    Rgb888::new(ri, gi, bi)
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _clocks = init_clocks_and_plls(
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

    let unipins = UnicornPins {
        led_blank: pins.gpio11.into_mode(),
        led_latch: pins.gpio10.into_mode(),
        led_clock: pins.gpio9.into_mode(),
        led_data: pins.gpio8.into_mode(),
        row_0: pins.gpio22.into_mode(),
        row_1: pins.gpio21.into_mode(),
        row_2: pins.gpio20.into_mode(),
        row_3: pins.gpio19.into_mode(),
        row_4: pins.gpio18.into_mode(),
        row_5: pins.gpio17.into_mode(),
        row_6: pins.gpio16.into_mode(),
    };

    let mut uni = pimoroni_unicorn_pio::Unicorn::new(&mut pio, sm0, unipins);
    // Buttons
    let btn_a = pins.gpio12.into_pull_up_input();
    let btn_b = pins.gpio13.into_pull_up_input();
    let btn_x = pins.gpio14.into_pull_up_input();
    let btn_y = pins.gpio15.into_pull_up_input();

    let mut led_pin = pins.led.into_push_pull_output();

    let sin = bsp::hal::rom_data::float_funcs::fsin::ptr();
    let sqrt = bsp::hal::rom_data::float_funcs::fsqrt::ptr();
    let t: F32Math = F32Math { sin, sqrt };

    let _ = led_pin.toggle();

    let mut hueshift_speed: i32 = 5;
    let mut plasma_scale: i32 = 22;
    let mut hue_shift: i32 = 0;

    let mut b: ButtonHandler =
        ButtonHandler::new(btn_a.into(), btn_b.into(), btn_x.into(), btn_y.into());

    // Give the plasma pattern some drift to make it appear more dynamic
    let mut x_drift = 0;
    let mut y_drift = 0;
    const X_DRIFT_RATE: u32 = 3;
    const Y_DRIFT_RATE: u32 = 1;
    const DRIFT_DIVISOR: f32 = 30.0;

    loop {
        let pressed = b.read();

        if pressed.a {
            plasma_scale += 1;
            info!("plasma_scale {:?}", plasma_scale);
        }
        if pressed.b {
            plasma_scale -= 1;
            if plasma_scale < 1 {
                plasma_scale = 1
            }
            info!("plasma_scale {:?}", plasma_scale);
        }

        if pressed.x {
            hueshift_speed += 1;
            info!("hueshift_speed {:?}", hueshift_speed);
        }

        if pressed.y {
            hueshift_speed -= 1;
            if hueshift_speed < 1 {
                hueshift_speed = 1
            }
            info!("hueshift_speed {:?}", hueshift_speed);
        }

        let plasma_scale_f32 = plasma_scale as f32 / 3.0;
        let hue_shift_f32 = hue_shift as f32 / 2000.0;
        let x_drift_f32 = x_drift as f32 / DRIFT_DIVISOR;
        let y_drift_f32 = y_drift as f32 / DRIFT_DIVISOR;
        for y in 0..pimoroni_unicorn_pio::HEIGHT {
            let y_scaled = y as f32 * plasma_scale_f32;
            for x in 0..pimoroni_unicorn_pio::WIDTH {
                let x_scaled = x as f32 * plasma_scale_f32;
                let pix = t.plasma_pixel(x_scaled + x_drift_f32, y_scaled + y_drift_f32);
                let value = hsv_to_rgb(pix + hue_shift_f32, 1.0, 1.0);
                uni.set_pixel(Point::new(x as i32, y as i32), value);
            }
        }
        uni.draw();
        hue_shift += hueshift_speed;
        x_drift += X_DRIFT_RATE;
        y_drift += Y_DRIFT_RATE;
        // Set a soft cap on drift or we run out of precision
        // TODO: something less jarring
        if x_drift > 10000 {
            x_drift = 0;
        }
        if y_drift > 10000 {
            y_drift = 0;
        }
    }
}

// End of file
