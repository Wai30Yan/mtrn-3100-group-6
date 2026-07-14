#![no_std]
#![no_main]
#![feature(str_as_str)]
#![feature(never_type)]
#![feature(unsafe_cell_access)]
#![feature(core_intrinsics)]
#![feature(generic_const_exprs)]
<<<<<<< HEAD
#![feature(clamp_magnitude)]
=======
>>>>>>> a5dec54 (Refactor the lidar to use read and write functions)

#[macro_use]
extern crate alloc;

use core::{cell::RefCell, f32, panic::PanicInfo, todo, unimplemented};

use cortex_m::prelude::_embedded_hal_timer_CountDown;
use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;
use na::{Isometry2, Rotation2, Translation2, Vector2};
use nb::block;
use stm32g4::stm32g431::{CorePeripherals, NVIC, Peripherals};
use stm32g4xx_hal::{
    delay::SYSTDelayExt,
    gpio::{GpioExt, PinState},
    hal::{delay::DelayNs, i2c::I2c},
    i2c::I2cExt,
    interrupt,
    pwm::PwmExt,
    pwr::{PwrExt, VoltageScale},
    rcc::{Config, PllConfig, PllMDiv, PllNMul, PllQDiv, PllRDiv, PllSrc, RccExt},
    time::{ExtU32, RateExtU32},
    timer::{Event, Timer},
    usb::{self, UsbBus},
};

use crate::{
    encoder::EncoderInstance, imu::Imu, lidar::Lidar, motion_manager::{Motion, MotionManager}, motor::Motor, serial::UsbSerial, state_observer::StateObserver,
};

extern crate nalgebra as na;

pub mod encoder;
pub mod imu;
pub mod lidar;
pub mod motion_manager;
pub mod motor;
pub mod serial;
pub mod state_observer;

pub type I2cBus<'a> = RefCell<&'a mut (dyn I2c<Error = stm32g4xx_hal::i2c::Error> + 'static)>;

const TIMESTEP_MS: u32 = 10;
const DT: f32 = (TIMESTEP_MS as f32) / 1000.0;

enum Task {
    StraightLineTracking,
    DrivingAndStopping,
    Turning,
    ChainingMovements(&'static str),
}

pub fn concat<T: Copy + Default, const A: usize, const B: usize>(
    a: &[T; A],
    b: &[T; B],
) -> [T; A + B] {
    let mut whole: [T; A + B] = [Default::default(); A + B];
    let (one, two) = whole.split_at_mut(A);
    one.copy_from_slice(a);
    two.copy_from_slice(b);
    whole
}

const LIDAR_ADDR_L: u8 = 0x27;
const LIDAR_ADDR_R: u8 = 0x28;
const LIDAR_ADDR_F: u8 = 0x29;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    unsafe { Peripherals::steal() }
        .GPIOC
        .bsrr()
        .write(|w| w.bs6().set_bit());

    // SAFETY: if we are panicking we have bigger issues, we just want to try
    // and scream out some diagnostics before dying.
    static mut DOUBLE_PANIC: bool = false;

    // Prevent us getting stuck in a loop if attempting to print panics
    if unsafe { !DOUBLE_PANIC } {
        print!("PANIC: {}\r\nAT: {:?}", info.message(), info.location());
    }

    unsafe {
        DOUBLE_PANIC = true;
    }

    loop {}
}

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
    /*
     * The first chunk of code is for low level initialisation of the hardware
     * to provide facilities such as printing, delays and memory allocation. On
     * an Arduino this would be done behind the scenes, but we want a bit more
     * control so we are explicit about how this is done.
     *
     * PLEASE DO NOT TOUCH THIS CODE since it can result in the STM32 appearing
     * unresponsive (ie. weird crashes before you have the ability to print data
     * to help you debug things).
     */

    // Allow dynamic memory allocation
    initialise_allocator();

    // Obtain access to MCU peripherals
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    // Put the MCU in the highest power mode. We want to run the CPU as fast a
    // possible and don't care about power consumption.
    let pwr = dp
        .PWR
        .constrain()
        .vos(VoltageScale::Range1 { enable_boost: true })
        .freeze();

    // Configure the clocks (core 144MHz)
    let mut rcc = dp.RCC.freeze(
        Config::pll().pll_cfg(PllConfig {
            mux: PllSrc::HSE(8.MHz()),
            m: PllMDiv::DIV_2,
            n: PllNMul::MUL_36,
            r: Some(PllRDiv::DIV_2),
            q: Some(PllQDiv::DIV_6),
            p: None,
        }),
        pwr,
    );

    // Required for USB
    rcc.enable_hsi48();

    // Set up pins
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    // Allow code to sleep, should only be used in the main loop
    let mut delay = cp.SYST.delay(&rcc.clocks);
    unsafe {
        UsbSerial::init(UsbBus::new(usb::Peripheral {
            usb: dp.USB,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
        }))
    };
    let mut led = gpioc.pc6.into_push_pull_output();
    // For USB tick, if the this is too slow then the buffers can get over
    // filled so run things at 1kHz to avoid this.
    Timer::new(dp.TIM7, &rcc.clocks)
        .start_count_down(1.millis())
        .listen(Event::TimeOut);
    unsafe { NVIC::unmask(interrupt::TIM7) };
    // Give time for hardware to initialise
    delay.delay_ms(500);
    print!("Initialisation Complete!\r\n");

    // Reset and enable the timer peripheral.
    rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(true));
    rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(false));
    rcc.apb1enr1().modify(|_, w| w.tim2en().bit(true));

    // updates on overflow happen automatically
    unsafe {
        // scale 144MHz to 1MHz
        dp.TIM2.psc().write(|w| w.psc().bits(144 - 1));
        // NOTE: do not disable updates, as the prescaler is loaded ON AN UPDATE EVENT
        // so the prescaler won't actually apply until e.g. counter overflows
        // (which we effectively set to happen immediately below)
        dp.TIM2.cnt().write(|w| w.cnt().bits(0xFFFFFFFF));
        // enable timer
        dp.TIM2.cr1().write(|w| w.cen().bit(true));
    }

    /*
     * Scary low level code is (mostly) done, this next section is equivlent to
     * the Arduino `setup` function.
     *
     * A key difference with the Arduino way of doing things is that objects
     * should be declared and initialised here. You will run into a host of
     * errors if you try to put things in globals outside of the main function.
     * This structure eliminates the design anti-pattern of having to call
     * seperate begin functions after creating objects; in Rust if an object
     * exists it should be in a valid and usable state.
     *
     * The STM32 Rust HAL makes extensive use of the rich type systems. A
     * downside of this is that it can be difficult to work out the specific
     * type you need without much experience. For my personal sanity I shall
     * create the required generic types.
     */

    /*
     * **NEW PIN MAPPINGS**
     *
     * Builtin LED -- PC6
     *
     * I2C -- I2C2: PA8 (SDA) + PA9 (SCL)
     *
     * Encoder L -- TIM4: PB6 + PB7
     * Encoder R -- TIM3: PA6 + PA4
     *
     * Motor L -- PA7 (PWM: TIM17) + PB11 (DIR)
     * Motor R -- PA2 (PWM: TIM15) + PB10 (DIR)
     *
     * LIDAR L -- PB12 (EN)
     * LIDAR R -- PB13 (EN)
     * LIDAR F -- PB14 (EN)
     */

    let mut period_timer = Timer::new(dp.TIM8, &rcc.clocks).start_count_down(TIMESTEP_MS.millis());

    let mut raw_i2c = dp.I2C2.i2c(
        (
            gpioa.pa8.into_alternate_open_drain().internal_pull_up(true),
            gpioa.pa9.into_alternate_open_drain().internal_pull_up(true),
        ),
        10.kHz(),
        &mut rcc,
    );
    let i2c_bus: I2cBus = RefCell::new(&mut raw_i2c);

    // Run at a higher frequency than the Arduino for smoother motion
    let mut motor_l_pwm = dp.TIM17.pwm(gpioa.pa7.into_alternate(), 24.kHz(), &mut rcc);
    let mut motor_r_pwm = dp.TIM15.pwm(gpioa.pa2.into_alternate(), 24.kHz(), &mut rcc);

    /*
     * Time to actually create our high level objects using the periherals
     * defined above. GPIO pins are far simpler to deal with so the
     * initialisation is performed in the main setup section.
     */

    let mut motor_left = Motor::new(
        &mut motor_l_pwm,
        gpiob.pb11.into_push_pull_output().erase(),
        true,
    );
    let mut motor_right = Motor::new(
        &mut motor_r_pwm,
        gpiob.pb10.into_push_pull_output().erase(),
        false,
    );

    // Set encoder pins to the correct mode
    gpiob.pb6.into_alternate::<2>().internal_pull_up(true);
    gpiob.pb7.into_alternate::<2>().internal_pull_up(true);
    gpioa.pa4.into_alternate::<2>().internal_pull_up(true);
    gpioa.pa6.into_alternate::<2>().internal_pull_up(true);

    let mut encoder_left = EncoderInstance::new(dp.TIM4, false, &mut rcc);
    let mut encoder_right = EncoderInstance::new(dp.TIM3, true, &mut rcc);

    let mut imu = Imu::new(&i2c_bus);

    let lidar_l_en = gpiob
        .pb12
        .into_open_drain_output_in_state(PinState::Low)
        .erase();
    let lidar_r_en = gpiob
        .pb13
        .into_open_drain_output_in_state(PinState::Low)
        .erase();
    let lidar_f_en = gpiob
        .pb14
        .into_open_drain_output_in_state(PinState::Low)
        .erase();
    let mut lidar_l = Lidar::new(&i2c_bus, lidar_l_en, LIDAR_ADDR_L);
    let mut lidar_r = Lidar::new(&i2c_bus, lidar_r_en, LIDAR_ADDR_R);
    let mut lidar_f = Lidar::new(&i2c_bus, lidar_f_en, LIDAR_ADDR_F);

    lidar_pin_l.set_high();
    let mut lidar_l = Lidar::new(&i2c_bus, LIDAR_ADDR_L);
    delay.delay_ms(100);
    print!("LIDAR L initialised\r\n");
    print!("LIDAR L distance: {} mm\r\n", lidar_l.get_distance());
    lidar_pin_l.set_low();

    lidar_pin_r.set_high();
    let mut lidar_r = Lidar::new(&i2c_bus, LIDAR_ADDR_R);
    delay.delay_ms(100);
    print!("LIDAR R initialised\r\n");
    print!("LIDAR R distance: {} mm\r\n", lidar_r.get_distance());
    lidar_pin_r.set_low();

    lidar_pin_f.set_high();
    let mut lidar_f = Lidar::new(&i2c_bus, LIDAR_ADDR_F);
    delay.delay_ms(100);
    print!("LIDAR F initialised\r\n");
    print!("LIDAR F distance: {} mm\r\n", lidar_f.get_distance());

    let mut observer = StateObserver::new();

    let mut motion_manager = MotionManager::new();

    // Let the state observer settle
    for _ in 1..450 {
        encoder_left.update();
        encoder_right.update();
        imu.update();
        observer.update(true, &imu, &encoder_left, &encoder_right);

        block!(period_timer.wait()).unwrap();
    }

    let task = Task::ChainingMovements("flfrrflr");

    match task {
        Task::StraightLineTracking => {
            motion_manager.set_target(Motion::Line {
                final_position: Translation2::new(1.05, 0.0),
                final_speed: 0.0,
            });
        }
        Task::DrivingAndStopping => {
            motion_manager.set_target(Motion::Line {
                final_position: Translation2::new(0.11, 0.0),
                final_speed: 0.0,
            });
        }
        Task::Turning => {}
        Task::ChainingMovements(_) => {}
    }

    let mut motion_index: usize = 0;

    /*
     * Because we are not building on top of any framework, everything goes into
     * the main function. There is no `loop` function like Arduino, but it is
     * pretty simple for us to define our own main loop.
     */
    loop {
        encoder_left.update();
        encoder_right.update();

        lidar_l.update();
        lidar_r.update();
        lidar_f.update();

        match task {
            Task::StraightLineTracking => {}
            Task::DrivingAndStopping => {
                if motion_manager.idle() {
                    // Relative to wall
                    motion_manager.set_target(todo!());
                }
            }
            Task::Turning => {
                motion_manager.set_target(Motion::Pivot {
                    rotation: Rotation2::new(-f32::consts::FRAC_PI_2),
                });
            }
            Task::ChainingMovements(cmd) => {
                if motion_manager.idle()
                    && let Some(c) = cmd.chars().nth(motion_index)
                {
                    motion_index += 1;
                    motion_manager.set_target(match c {
                        'l' => Motion::Pivot {
                            rotation: (motion_manager.pose().rotation
                                * Rotation2::new(f32::consts::FRAC_PI_2))
                            .into(),
                        },
                        'r' => Motion::Pivot {
                            rotation: (motion_manager.pose().rotation
                                * Rotation2::new(-f32::consts::FRAC_PI_2))
                            .into(),
                        },
                        'f' => Motion::Line {
                            final_position: (motion_manager.pose()
                                * Isometry2::new(Vector2::new(0.18, 0.0), 0.0))
                            .translation,
                            final_speed: 0.0,
                        },
                        _ => {
                            unimplemented!("Invalid command")
                        }
                    });
                }
            }
        }

        let desired = motion_manager.update(observer.pose());
        let (wl, wr) = desired.to_wheel_velocities();
        print!(
            "Ll: {} mm\tLr: {} mm\tLf: {} mm\r\n",
            lidar_l.distance(),
            lidar_r.distance(),
            lidar_f.distance()
        );

        // print!("{:?}\r\n", observer.pose());
        // delay.delay_ms(1);
        print!("{:?}\r\n", motion_manager.pose());
        // delay.delay_ms(1);
        // print!("{:?}\r\n", desired.to_wheel_velocities());

        motor_left.set_speed(wl);
        motor_right.set_speed(wr);

        /*
         * This MCU is extreme overkill so it is fine to assume that we can
         * finish all our work within 10ms, wait for the remaining time to pass
         * so things run on a consistent schedule.
         *
         * As a result we do not need to run the control loop in an interupt.
         */
        block!(period_timer.wait()).unwrap();
    }
}
