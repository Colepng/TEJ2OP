#![no_std]
#![no_main]

use core::result::Result::{Err, Ok};
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
use embedded_hal::adc::OneShot;
use embedded_hal::PwmPin;
use fugit::ExtU32;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::Adc;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[macro_use]
extern crate alloc;

use alloc::str::from_utf8;
use alloc::vec::*;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
    // Set to watchdog to reset if it's not reloaded within 1.05 seconds, and start it
    watchdog.start(1.secs());

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

    // init pwm
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    // Setup adc
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sensor
    let mut temp_sense = adc.enable_temp_sensor();
    let mut offset: f64 = 26000.0;

    let mut temperature_adc_counts: u16;
    let mut adc_volts: f64;
    let mut temp: f64;
    let mut led_brightness: u16;

    // let mut led_pin = pins.led.into_push_pull_output();
    let mut text: Vec<u8> = Vec::new();
    let mut prompt: bool = false;
    let mut command: &str;
    let mut args: Vec<&str>;

    let mut led_enabled: bool = true;

    loop {
        // Feed the watchdog
        watchdog.feed();
        // https://electrocredible.com/raspberry-pi-pico-temperature-sensor-tutorial/

        // Reads the counts from the on-chip temperature sensor
        // calculates the actual temperature in celsius
        temperature_adc_counts = adc.read(&mut temp_sense).unwrap();
        adc_volts = temperature_adc_counts as f64 * (3.3 / 4095.0);
        temp = 27.0 - ((adc_volts - 0.706) / 0.001721);
        led_brightness = ((temp * 1000.0) - offset) as u16;

        if usb_dev.poll(&mut [&mut serial]) {
            let mut usb_buffer: [u8; 64] = [0u8; 64];

            if !prompt {
                // TODO! handle error
                let _ = serial.write(b"Enter Command: ");
                prompt = true;
            }
            match serial.read(&mut usb_buffer) {
                Ok(0) => {}
                Err(_) => {}
                Ok(n_bytes_read) => {
                    text.push(usb_buffer[0]);

                    let mut wr_ptr = &usb_buffer[..n_bytes_read];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            Err(_) => break,
                        }
                    }

                    if text[text.len() - 1] == b'\r' {
                        prompt = false;
                    }
                    if !prompt {
                        command = from_utf8(
                            &text[..text
                                .iter()
                                .position(|x| x == &b' ' || x == &b'\r' || x == &b'\n')
                                .unwrap()],
                        )
                        .unwrap();

                        args = from_utf8(
                            &text[text
                                .iter()
                                .position(|x| x == &b' ' || x == &b'\r' || x == &b'\n')
                                .unwrap()..],
                        )
                        .unwrap()
                        .trim()
                        .split(' ')
                        .collect::<Vec<&str>>();

                        match command {
                            "temp" => {
                                let _ = serial.write(format!("temperature {temp}\n").as_bytes());
                            }
                            "flash" => {
                                let _ = serial.write(b"entering flash mode");
                                hal::rom_data::reset_to_usb_boot(25, 0);
                            }
                            "led" => {
                                match args[0] {
                                    "on" => {
                                        led_enabled = true;
                                    }
                                    "off" => {
                                        led_enabled = false;
                                        channel.set_duty(0);
                                    }
                                    "offset" => {
                                        if let Ok(x) = args[1].parse::<f64>() {
                                            offset = x * 1000.0;
                                        } else {
                                            let _ =
                                                serial.write(b"please enter a correct offset\n");
                                        }
                                    }
                                    // TODO! add help
                                    _ => {}
                                }
                            }
                            "" => {}
                            "test-watchdog" => loop {},
                            _ => {
                                let _ = serial
                                    .write(format!("command not found: {command}\n").as_bytes());
                            }
                        }

                        text = Vec::new();
                    }
                }
            }

            if led_enabled {
                channel.set_duty(led_brightness);
            }
        }
    }
}
