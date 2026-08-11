use core::f32;

use stm32g4::{Periph, stm32g431::tim3};
use stm32g4xx_hal::rcc::{Enable, Rcc, Reset};

pub trait Encoder {
    // Get the current position of the encoder in radians
    fn position(&self) -> f32;
}

type Timer<const A: usize> = Periph<tim3::RegisterBlock, A>;
pub struct EncoderInstance<const A: usize>
where
    Timer<A>: Enable + Reset,
{
    timer: Timer<A>,
    count_upper: i16,
    count_lower: u16,
    invert: bool,
}

// 2800 CPR encoder (double since we are using both edges)
const ENCODER_FACTOR: f32 = 2.0 * f32::consts::PI / 2800.0;

impl<const A: usize> EncoderInstance<A>
where
    Timer<A>: Enable + Reset,
{
    pub fn new(timer: Timer<A>, invert: bool, rcc: &mut Rcc) -> Self {
        Timer::<A>::enable(rcc);
        Timer::<A>::reset(rcc);

        timer.ccmr1_input().write(|w| w.cc1s().ti1().cc2s().ti2());
        timer.ccer().write(|w| {
            w.cc1p()
                .clear_bit()
                .cc1np()
                .clear_bit()
                .cc2p()
                .clear_bit()
                .cc2np()
                .clear_bit()
        });
        timer
            .smcr()
            .write(|w| unsafe { w.sms().bits(3).sms_3().clear_bit() });
        timer.cr1().write(|w| w.cen().set_bit());

        Self {
            timer,
            count_upper: 0,
            count_lower: 0,
            invert,
        }
    }

    pub fn update(&mut self) {
        let raw_count = self.timer.cnt().read().cnt().bits();
        // Check for overflow/underflow, the motor does not spin fast enough for
        // us to be forced to do this with interupts
        if raw_count.abs_diff(self.count_lower) > (u16::MAX / 2) {
            if raw_count < self.count_lower {
                // Overflow
                self.count_upper += 1;
            } else {
                // Underflow
                self.count_upper -= 1;
            }
        }
        self.count_lower = raw_count;
    }
}

impl<const A: usize> Encoder for EncoderInstance<A>
where
    Timer<A>: Enable + Reset,
{
    fn position(&self) -> f32 {
        ((self.count_upper as i32) * (u16::MAX as i32) + (self.count_lower as i32)) as f32
            * ENCODER_FACTOR
            * if self.invert { -1.0 } else { 1.0 }
    }
}
