use stm32g4xx_hal::{
    gpio::{AnyPin, Output, PinState, PushPull},
    hal_02::PwmPin,
};

use crate::DT;

pub struct Motor<'a> {
    pwm: &'a mut dyn PwmPin<Duty = u16>,
    dir: AnyPin<Output<PushPull>>,
    invert: bool,
    prev_setpoint: f32,
}

const KV: f32 = 0.040;
const KS: f32 = 0.018;
const MIN_DUTY: u16 = 16;
const MAX_SLEW: f32 = 50.0 * DT;

impl<'a> Motor<'a> {
    pub fn new(
        pwm: &'a mut dyn PwmPin<Duty = u16>,
        dir: AnyPin<Output<PushPull>>,
        invert: bool,
    ) -> Self {
        pwm.set_duty(0);
        pwm.enable();
        Self {
            pwm,
            dir,
            invert,
            prev_setpoint: 0.0,
        }
    }

    /// Set the target wheel velocity in rad/s using open loop control
    pub fn set_speed(&mut self, mut setpoint: f32) {
        setpoint = self.prev_setpoint + (setpoint - self.prev_setpoint).clamp_magnitude(MAX_SLEW);
        self.prev_setpoint = setpoint;
        let effort = setpoint * KV + KS.copysign(setpoint);

        self.dir.set_state(if self.invert ^ (effort > 0.0) {
            PinState::High
        } else {
            PinState::Low
        });

        let duty = effort.clamp_magnitude(1.0).abs() * (self.pwm.get_max_duty() as f32);

        self.pwm.set_duty(if (duty as u16) < MIN_DUTY {
            0
        } else {
            duty as u16
        });
    }
}
