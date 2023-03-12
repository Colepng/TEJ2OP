#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal::pac;
use rp_pico::hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use core::result::Result::{Ok, Err};

#[macro_use]
extern crate alloc;

use embedded_alloc::Heap;
use alloc::vec::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();


fn compare(vector: &Vec<u8>, text: &str) -> bool {
    if vector.len() >= text.len() {
        if &vector[vector.len()-text.len()..] == text.as_bytes() {
            return true;
        }
    }
    false
}


#[entry]
fn main() -> ! {

        // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    
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

    #[cfg(feature = "rp2040-e5")]
    {
        let sio = hal::Sio::new(pac.SIO);
        let _pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

        // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let sio = hal::Sio::new(pac.SIO);
   
    // Set the pins up according to their function on the pi pico 
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let mut text: Vec<u8> = vec![];
    
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            let serial_read = serial.read(&mut buf[..]);

            match serial_read {
                Ok(0) => {
                    serial.write(b"empty buffer");
                }
                Err(_e) => {
                    //  Do nothing
                }
                Ok(nums_of_bytes_read) => {
                    text.push(buf[0]);
                    if compare(&text, "ON\r") {
                        led_pin.set_high();
                    }
                    else if compare(&text, "OFF\r") {
                        led_pin.set_low();
                    }

                    let mut wr_ptr = &buf[..nums_of_bytes_read];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                                Ok(len) => wr_ptr = &wr_ptr[len..],
                                // On error, just drop unwritten data.
                                // One possible error is Err(WouldBlock), meaning the USB write buffer is full.
                                Err(_) => break,
                        }
                    }
                }
            }
        }
    }
}

