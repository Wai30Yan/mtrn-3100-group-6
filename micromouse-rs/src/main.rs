#![no_std]
#![no_main]

use core::panic::PanicInfo;

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;
use stm32g4::stm32g431::{Peripherals};
use stm32g4xx_hal::{
    gpio::GpioExt, pwr::{PwrExt, VoltageScale}, rcc::{Config, PllConfig, PllMDiv, PllNMul, PllQDiv, PllRDiv, PllSrc, RccExt}, time::RateExtU32, usb::{self, UsbBus}
};
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // hprintln!("{}", _info.message());
    loop {}
}

// Global allocator -- required by canadensis.
#[global_allocator]
static HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 0x4000; // 16 KiB
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

#[entry]
fn main() -> ! {
    initialise_allocator();

    let dp = Peripherals::take().unwrap();
    let pwr = dp
        .PWR
        .constrain()
        .vos(VoltageScale::Range1 { enable_boost: true })
        .freeze();

    let mut rcc = dp.RCC.freeze(
        Config::pll().pll_cfg(PllConfig {
            mux: PllSrc::HSE(8.MHz()),
            m: PllMDiv::DIV_1,
            n: PllNMul::MUL_36,
            r: Some(PllRDiv::DIV_2),
            q: Some(PllQDiv::DIV_6),
            p: None,
        }),
        pwr,
    );

    // Set up pins.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod: stm32g4xx_hal::gpio::gpiod::Parts = dp.GPIOD.split(&mut rcc);

    let mut led = gpioc.pc6.into_push_pull_output();

    loop {
        led.set_high();
        delay(1024*1024*64);
        led.set_low();
        delay(1024*1024*64);
    }
}
