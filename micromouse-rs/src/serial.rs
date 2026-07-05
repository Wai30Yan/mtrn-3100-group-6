/*
 * I have committed a lot of crimes in this file and it does thing quite
 * differently to the codebase. In other words is not advisable to use this file
 * as an example and it is not particularly important to understand.
 *
 * At a high level this periodically handles USB stuff in the background to
 * allow us to read from and write to a serial monitor. This goes against the
 * usual Rust way of doing things since we want to be able to interact with this
 * USB object from anywhere (so we can print from anywhere for debugging
 * purposes) so we need to do a couple tricks allow this.
 */

use core::{
    cell::UnsafeCell,
    mem::{MaybeUninit, swap},
};

use alloc::{
    collections::vec_deque::VecDeque,
    string::{String, ToString},
};

use cortex_m::interrupt::Mutex;
use nb::Error::WouldBlock;
use stm32g4xx_hal::{
    gpio::{self, AF14},
    interrupt,
    usb::{self, UsbBus},
};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

use alloc::boxed::Box;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

type UsUsbBus = UsbBus<usb::Peripheral<gpio::PA11<AF14>, gpio::PA12<AF14>>>;

pub struct UsbSerial<'a> {
    serial: SerialPort<'a, UsUsbBus>,
    usb_dev: UsbDevice<'a, UsUsbBus>,

    read_buf: VecDeque<u8>,
    write_buf: VecDeque<u8>,
}

static SERIAL: Mutex<UnsafeCell<MaybeUninit<UsbSerial<'static>>>> =
    Mutex::new(UnsafeCell::new(MaybeUninit::uninit()));

impl UsbSerial<'static> {
    fn new(usb_bus: &'static UsbBusAllocator<UsUsbBus>) -> Self {
        let serial = SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(USB_CLASS_CDC)
            .build();

        Self {
            serial,
            usb_dev,
            read_buf: VecDeque::new(),
            write_buf: VecDeque::new(),
        }
    }

    /// # Safety
    /// very bad things will happen if this is called twice or it is not called before the read_line/write global methods
    pub unsafe fn init(usb_bus: UsbBusAllocator<UsUsbBus>) {
        cortex_m::interrupt::free(|cs| unsafe {
            SERIAL
                .borrow(cs)
                .as_mut_unchecked()
                .write(Self::new(Box::leak(Box::new(usb_bus))));
        });
    }

    pub fn tick(&mut self) {
        if !self.usb_dev.poll(&mut [&mut self.serial]) {
            return;
        }

        let slices = self.write_buf.as_slices();
        if let Ok(n) = self.serial.write(slices.0) {
            if n == slices.0.len()
                && let Ok(k) = self.serial.write(slices.1)
            {
                self.write_buf.drain(0..n + k);
            }
            self.write_buf.drain(0..n);
        }

        let mut buf = [0u8; 256];
        if let Ok(n) = self.serial.read(&mut buf) {
            self.read_buf.extend(buf[0..n].iter());
        }
    }

    pub fn read_line(&mut self) -> nb::Result<String, !> {
        if let Some(x) = self.read_buf.iter().position(|&c| c == b'\n') {
            let mut rest = self.read_buf.split_off(x + 1);
            swap(&mut rest, &mut self.read_buf);
            Ok(String::from_utf8_lossy(rest.make_contiguous().as_slice()).to_string())
        } else {
            nb::Result::Err(WouldBlock)
        }
    }

    pub fn write(&mut self, str: String) {
        self.write_buf.extend(str.bytes());
    }
}

#[interrupt]
fn TIM7() {
    cortex_m::interrupt::free(|cs| unsafe {
        SERIAL
            .borrow(cs)
            .as_mut_unchecked()
            .assume_init_mut()
            .tick();
    })
}

pub fn read_line() -> nb::Result<String, !> {
    cortex_m::interrupt::free(|cs| unsafe {
        SERIAL
            .borrow(cs)
            .as_mut_unchecked()
            .assume_init_mut()
            .read_line()
    })
}

pub fn write(str: String) {
    cortex_m::interrupt::free(|cs| unsafe {
        SERIAL
            .borrow(cs)
            .as_mut_unchecked()
            .assume_init_mut()
            .write(str)
    })
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        serial::write(format!($($arg)*))
    }
}
