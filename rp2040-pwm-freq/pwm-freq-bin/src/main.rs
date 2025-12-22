#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::pwm::SetDutyCycle;
use panic_probe as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry, pac,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::delay::DelayNs;
use rp2040_hal as hal;

use pwm_freq::PwmFreq;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sysclkfreq = clocks.system_clock.freq().to_Hz();
    let mut delay = rp2040_hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm: &mut rp2040_hal::pwm::Slice<rp2040_hal::pwm::Pwm4, rp2040_hal::pwm::FreeRunning> =
        &mut pwm_slices.pwm4;
    pwm.set_top(0);
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to GPIO 25
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio25);
    channel.output_to(pins.gpio9);
    // Set duty cycle to 50%
    delay.delay_ns(100_000_000);
    // Start at the minimum acceptable frequency.
    let mut target_freq = 8;
    // Can't possibly go higher than the sysclk frequency
    let target_max = sysclkfreq;
    loop {
        let pwm_freq = PwmFreq::new(sysclkfreq, target_freq).unwrap();
        let pwm_config = pwm_freq.get_config();
        let calculated_freq = pwm_freq.get_actual_freq();
        // there may be some frequency glitch when changing frequency (between when we set the CC via set_duty_cycle and when we set the top)
        // and when we change the fractional divider. you should disable PWM output if this is a problem in your application

        // Set CC to half of top to get 50% duty cycle
        pwm.channel_b.set_duty_cycle(pwm_config.top / 2).unwrap();
        pwm.set_top(pwm_config.top);
        pwm.set_div_int(pwm_config.div.0);
        pwm.set_div_frac(pwm_config.div.1);
        // delay.delay_ns(100_000_000);
        info!(
            "target_freq: {}, actual_freq: {}",
            target_freq, calculated_freq
        );

        target_freq += 1;
        // Wrap around to the minimum frequency when we reach the maximum
        if target_freq > target_max {
            target_freq = 8;
        }

        // Update the pwm settings every 1/2 second
        delay.delay_ns(500_000_000);
    }
}

// End of file
