#![no_std]
#![no_main]

use embedded_hal::PwmPin;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal::pac;
use rp_pico::hal;
use cortex_m::prelude::*;
use fugit::ExtU32;

use nb;

const LOW: u16 = 0;
const HIGH: u16 = 25000;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    
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
    
    // init pwm
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    // Blink the LED at 1 Hz
    loop {
        // Ramp brightness up
        for i in (LOW..=HIGH).skip(100) {
            channel.set_duty(i);
            count_down.start(8.micros());
            let _ = nb::block!(count_down.wait());
        }

        // Ramp brightness down
        for i in (LOW..=HIGH).rev().skip(100) {
            channel.set_duty(i);
            count_down.start(8.micros());
            let _ = nb::block!(count_down.wait());
        }

        count_down.start(1.secs());
        let _ = nb::block!(count_down.wait());
    }
}

