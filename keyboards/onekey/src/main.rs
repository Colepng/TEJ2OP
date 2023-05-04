#![no_std]
#![no_main]

use cortex_m::prelude::{_embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable};
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use panic_halt as _;
use rp_pico::hal::gpio::bank0;
use rp_pico::hal::gpio::Input;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::Pin;
use rp_pico::hal::gpio::PullDown;
use rp_pico::hal::gpio::Readable;
use rp_pico::hal::pac::interrupt;
use rp_pico::{entry, hal};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

// USB Human Interface Device (HID) Class support
// use usbd_hid::descriptor::generator_prelude::*;
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

    unsafe {
        USB_HID = Some(usb_hid);
    }

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

    // let mut led = pins.led.into_push_pull_output();

    let power_row: bool = false;

    let keys = [
        [0x04, 0x05, 0x06], 
        [0x07, 0x08, 0x09],
    ];

    // do this for the rest of output and input tommorow in ics
    enum OutputPins {
        GP28(Pin<bank0::Gpio28, Output<Readable>>),
        GP26(Pin<bank0::Gpio26, Output<Readable>>),
        GP17(Pin<bank0::Gpio17, Output<Readable>>),
    }
    enum InputPins {
        GP15(Pin<bank0::Gpio15, Input<PullDown>>),
        GP16(Pin<bank0::Gpio16, Input<PullDown>>),
    }

    if power_row {
        let mut rows = [
            OutputPins::GP28(pins.gpio28.into_readable_output()),
            OutputPins::GP26(pins.gpio26.into_readable_output()),
            OutputPins::GP17(pins.gpio17.into_readable_output()),
        ];
        let cols = [
            InputPins::GP16(pins.gpio16.into_pull_down_input()),
            InputPins::GP15(pins.gpio15.into_pull_down_input()),
        ];

        loop {
            // feed watchdog
            watchdog.feed();
            // moving this outside of the loop can make this more effiecnt by checking if the key is pressed and not over writing it
            let mut keycodes: [u8; 6] = [0x00; 6];
            let mut index: usize = 0;
            for (row, pin) in rows.iter_mut().enumerate() {
                match pin {
                    OutputPins::GP17(x) => x.set_high().unwrap(),
                    OutputPins::GP26(x) => x.set_high().unwrap(),
                    OutputPins::GP28(x) => x.set_high().unwrap(),
                }
                for (col, pin) in cols.iter().enumerate() {
                    match pin {
                        InputPins::GP15(x) => {
                            if index <= 6 && x.is_high().unwrap() {
                                keycodes[index] = keys[row][col];
                                index += 1;
                            }
                        }
                        InputPins::GP16(x) => {
                            if index <= 6 && x.is_high().unwrap() {
                                keycodes[index] = keys[row][col];
                                index += 1;
                            }
                        }
                    }
                }
                match pin {
                    OutputPins::GP17(x) => x.set_low().unwrap(),
                    OutputPins::GP26(x) => x.set_low().unwrap(),
                    OutputPins::GP28(x) => x.set_low().unwrap(),
                }
            }
            let report = KeyboardReport {
                modifier: 0x00,
                reserved: 0x00,
                leds: 0x00,
                keycodes: keycodes,
            };
            push_keyboard_inputs(report).ok().unwrap_or(0);
        }
    } else {
        let mut cols = [
            OutputPins::GP28(pins.gpio28.into_readable_output()),
            OutputPins::GP26(pins.gpio26.into_readable_output()),
            OutputPins::GP17(pins.gpio17.into_readable_output()),
        ];
        let rows = [
            InputPins::GP16(pins.gpio16.into_pull_down_input()),
            InputPins::GP15(pins.gpio15.into_pull_down_input()),
        ];
        loop {
            // feed watchdog
            watchdog.feed();
            // moving this outside of the loop can make this more effiecnt by checking if the key is pressed and not over writing it
            let mut keycodes: [u8; 6] = [0x00; 6];
            let mut index: usize = 0;
            for (col, pin) in cols.iter_mut().enumerate() {
                match pin {
                    OutputPins::GP17(x) => x.set_high().unwrap(),
                    OutputPins::GP26(x) => x.set_high().unwrap(),
                    OutputPins::GP28(x) => x.set_high().unwrap(),
                }
                for (row, pin) in rows.iter().enumerate() {
                    match pin {
                        InputPins::GP15(x) => {
                            if index <= 6 && x.is_high().unwrap() {
                                keycodes[index] = keys[row][col];
                                index += 1;
                            }
                        }
                        InputPins::GP16(x) => {
                            if index <= 6 && x.is_high().unwrap() {
                                keycodes[index] = keys[row][col];
                                index += 1;
                            }
                        }
                    }
                }
                match pin {
                    OutputPins::GP17(x) => x.set_low().unwrap(),
                    OutputPins::GP26(x) => x.set_low().unwrap(),
                    OutputPins::GP28(x) => x.set_low().unwrap(),
                }
            }
            let report = KeyboardReport {
                modifier: 0x00,
                reserved: 0x00,
                leds: 0x00,
                keycodes: keycodes,
            };
            push_keyboard_inputs(report).ok().unwrap_or(0);
        }
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
