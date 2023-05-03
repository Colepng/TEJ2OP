#![no_std]
#![no_main]

use cortex_m::prelude::{_embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use panic_halt as _;
use rp_pico::hal::gpio::pin::Pin;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::Clock;
use rp_pico::{entry, hal};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

// USB Human Interface Device (HID) Class support
// use usbd_hid::descriptor::generator_prelude::*;
// use usbd_hid::descriptor::MouseReport;
use cortex_m::peripheral::NVIC;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<hid_class::HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    // setup peripherals
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // setup watchdog
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    watchdog.start(1.secs());

    // setup serial input/output
    let sio = hal::Sio::new(pac.SIO);

    // setup clock at 125Mhz
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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB Communications Class Device driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Setup usb hid class
    let usb_hid = hid_class::HIDClass::new_ep_in_with_settings(
        bus_ref,
        KeyboardReport::desc(),
        60,
        hid_class::HidClassSettings {
            subclass: hid_class::HidSubClass::NoSubClass,
            config: hid_class::ProtocolModeConfig::ForceReport,
            locale: hid_class::HidCountryCode::US,
            protocol: hid_class::HidProtocol::Keyboard,
        },
    );

    // let usb_hid = hid_class::HIDClass::new(bus_ref, KeyboardReport::desc(), 60);

    unsafe {
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    // let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
    //     .manufacturer("Cole corp")
    //     .product("One key keyboard")
    //     .serial_number("TEST")
    //     .device_class(3)
    //     .build();Gen

    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Cole corp")
        .product("One Key")
        .serial_number("1")
        .device_class(0)
        .build();

    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        hal::pac::NVIC::unmask(hal::pac::interrupt::USBCTRL_IRQ);
    };

    let mut pin17 = pins.gpio17.into_readable_output();
    pin17.set_high().unwrap();
    let pin16 = pins.gpio16.into_pull_down_input();
    let mut led = pins.led.into_push_pull_output();
    // let core = hal::pac::CorePeripherals::take().unwrap();
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        // feed watchdog
        watchdog.feed();
        // usb_dev.poll(&mut [&mut usb_hid]);
        // if pin16.is_high().unwrap() {
        //     led.set_high().unwrap()
        //     // let _ = &mut usb_hid.push_input(&KeyboardReport { modifier: 0x00, reserved: 0x00, leds: 0x00, keycodes: [0x04, 0x00, 0x00, 0x00, 0x00, 0x00]});
        // } else {
        //     led.set_low().unwrap()
        // }
        //    led.set_high().unwrap()
        // } else {
        //    led.set_low().unwrap()
        // }
        //temp
        // if temp {
        // delay.delay_ms(100);
        if pin16.is_high().unwrap() {
            let b = KeyboardReport {
                modifier: 0x00,
                reserved: 0x00,
                leds: 0x00,
                keycodes: [0x5, 0x00, 0x00, 0x00, 0x00, 0x00],
            };
            push_keyboard_inputs(b)
            .ok()
            .unwrap_or(0);
            led.set_high().unwrap();
        } else {
            let b = KeyboardReport {
                modifier: 0x00,
                reserved: 0x00,
                leds: 0x00,
                keycodes: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            };
            push_keyboard_inputs(b)
            .ok()
            .unwrap_or(0);
            led.set_low().unwrap();
        }
        // delay.delay_ms(100);
        // let a = KeyboardReport {
        //     modifier: 0x00,
        //     reserved: 0x00,
        //     leds: 0x00,
        //     keycodes: [0x00; 6],
        // };
        //
        // push_keyboard_inputs(a)
        // .ok()
        // .unwrap_or(0);
        // temp = false;
        // } else {
        // delay.delay_ms(100);
        //
        // push_keyboard_inputs(KeyboardReport {
        //     modifier: 0x00,
        //     reserved: 0x00,
        //     leds: 0x00,
        //     keycodes: [0x05, 0x00, 0x00, 0x00, 0x00, 0x00],
        // })
        // .ok()
        // .unwrap_or(0);
        // led.set_low().unwrap();
        // temp = true;
        // }
    }
}

fn push_keyboard_inputs(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_hid = USB_HID.as_mut().unwrap();
    let usb_device = USB_DEVICE.as_mut().unwrap();
    usb_device.poll(&mut [usb_hid]);
}
