#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal;
use fugit::ExtU32;
use cortex_m::prelude::*;
use nb;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    
    // Set up the watchdog driver
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Setup up a clock at 125MHz
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    let sio = hal::Sio::new(pac.SIO);
   
    // Set the pins up according to their function on the pi pico 
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // Blink the LED at 1 Hz
    loop {
        // turns the led on
        led_pin.set_high().unwrap();
        // start a count down
        count_down.start(500.millis());
        let _ = nb::block!(count_down.wait());

        // turns the led off
        led_pin.set_low().unwrap();

        // LED off, and wait for 500ms
        led_pin.set_low().unwrap();
        count_down.start(5.secs());
        let _ = nb::block!(count_down.wait());
    }
}

