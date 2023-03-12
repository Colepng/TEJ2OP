// create a no std environment for embedded programming
#![no_std]
#![no_main]

use rp_pico::{entry};
use rp_pico::hal::pac;
use rp_pico::hal;
use panic_halt as _;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// sets up an allocator
#[macro_use]
extern crate alloc;

use embedded_alloc::Heap;
use alloc::vec::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
    let mut _count_down = timer.count_down();
    
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS
    ));

    // Set up USB Communications Class Device driver
    let mut _serial = SerialPort::new(&usb_bus);
   
    // Create a USB device with a fake vendor ID and product ID
    let mut _usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    let sio = hal::Sio::new(pac.SIO);

    // Set the pins according to their function on the pi pico
    let _pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    loop {

    }
}
