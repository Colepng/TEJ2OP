// create a no std environment for embedded programming
#![no_std]
#![no_main]

use rp_pico::{entry, hal::timer::CountDown};
use rp_pico::hal::pac;
use rp_pico::hal;
use panic_halt as _;
use cortex_m::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use crate::hal::gpio::bank0::Gpio25;
use crate::hal::gpio::Pin;
use crate::hal::gpio::PushPull;
use crate::hal::gpio::Output;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use fugit::ExtU32;
use nb;

// sets up an allocator
#[macro_use]
extern crate alloc;

use embedded_alloc::Heap;
use alloc::vec::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();

// function used to compare a 
fn compare(vector: &Vec<u8>, text: &str) -> bool {
    if vector.len() >= text.len() {
        if &vector[vector.len()-text.len()..] == text.as_bytes() {
            return true;
        }
    }
    false
}

fn wait(count_down: &mut CountDown, wait_for:u32) {
    count_down.start(wait_for.millis());
    let _ = nb::block!(count_down.wait());
}

const UNIT: u32 = 250;

fn dot(led: &mut Pin<Gpio25, Output<PushPull>>, count_down: &mut CountDown) {
    led.set_high();
    wait(count_down, UNIT);
    led.set_low();
}

fn dash(led: &mut Pin<Gpio25, Output<PushPull>>, count_down: &mut CountDown){
    led.set_high();
    wait(count_down, UNIT*3);
    led.set_low();
}

struct MorseCode();

impl MorseCode {
    fn a(&self, count_down: &mut CountDown, led: &mut Pin<Gpio25, Output<PushPull>>) {
        dot(led, count_down);
        wait(count_down, UNIT);
        dash(led, count_down);
    }

    fn b(&self, count_down: &mut CountDown, led: &mut Pin<Gpio25, Output<PushPull>>) {
        dash(led, count_down);
        wait(count_down, UNIT);
        dot(led, count_down);
        wait(count_down, UNIT);
        dot(led, count_down);
        wait(count_down, UNIT);
        dot(led, count_down);
    }
}
#[entry]
fn main() -> ! {
    // Initialize the allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    
    // Set up the watchdog driver
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    
    // Setup a clock at 125MHz
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

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();
    
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS
    ));

    // Set up USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);
   
    // Create a USB device with a fake vendor ID and product ID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    let sio = hal::Sio::new(pac.SIO);

    // Set the pins according to their function on the pi pico
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let mut collect_buf: Vec<u8> = vec![];

    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buffer = [0u8; 64];
            
            match serial.read(&mut buffer[..]) {
                Ok(0) => {
                    // Buffer is empty
                }
                Err(_e) => {
                    // An error has occurred 
                }
                Ok(_nums_of_bytes_read) => {
                    // Since the number of bytes read is always 1 I am just appending the first
                    // element of buffer
                    collect_buf.push(buffer[0]);
                    if compare(&collect_buf, "a") {
                        MorseCode().a(&mut count_down, &mut led_pin);
                        wait(&mut count_down, UNIT*3);
                    } else if compare(&collect_buf, "b") {
                        MorseCode().b(&mut count_down, &mut led_pin);
                        wait(&mut count_down, UNIT*3);
                    }
                }
            }
        }
    }
}
