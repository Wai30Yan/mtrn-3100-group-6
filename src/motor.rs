use stm32g4xx_hal::{
    gpio::{AnyPin, Output, PinState, PushPull},
    hal_02::PwmPin,
};

pub struct Motor<'a> {
    pwm: &'a mut dyn PwmPin<Duty = u16>,
    dir: AnyPin<Output<PushPull>>,
    invert: bool,
}

const KV: f32 = 0.040;
const KS: f32 = 0.006;
const MIN_DUTY: u16 = 256;

impl<'a> Motor<'a> {
    pub fn new(
        pwm: &'a mut dyn PwmPin<Duty = u16>,
        dir: AnyPin<Output<PushPull>>,
        invert: bool,
    ) -> Self {
        pwm.set_duty(0);
        pwm.enable();
        Self { pwm, dir, invert }
    }

    /// Set the target wheel velocity in rad/s using open loop control
    pub fn set_speed(&mut self, v: f32) {
        self.dir.set_state(if self.invert ^ (v > 0.0) {
            PinState::High
        } else {
            PinState::Low
        });

        let duty = ((v.abs() * KV + KS) * (self.pwm.get_max_duty() as f32)) as u16;
        self.pwm.set_duty(if duty < MIN_DUTY { 0 } else { duty });
    }
}
