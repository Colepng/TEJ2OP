#![no_std]
#![no_main]

use core::result::Result::{Err, Ok};
use cortex_m::Peripherals;
use embedded_hal::adc::OneShot;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::Adc;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[macro_use]
extern crate alloc;

use alloc::vec::*;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

fn compare(vector: &Vec<u8>, text: &str) -> bool {
    if vector.len() >= text.len() && &vector[vector.len() - text.len()..] == text.as_bytes() {
        return true;
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

    // Setup adc
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sensor
    let mut temp_sense = adc.enable_temp_sensor();

    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        // https://electrocredible.com/raspberry-pi-pico-temperature-sensor-tutorial/
        let temperature_adc_counts: u16 = adc.read(&mut temp_sense).unwrap();
        let adc_volts: f64 = temperature_adc_counts as f64 * (3.3 / 4095.0);
        let temp: f64 = 27.0 - ((adc_volts - 0.706)/0.001721);
        if usb_dev.poll(&mut [&mut serial]) {
            let _ = serial.write(format!("temperature_adc_counts {temperature_adc_counts}\n").as_bytes());
            let _ = serial.write(format!("temperature {temp}\n").as_bytes());
        }
    }
}
