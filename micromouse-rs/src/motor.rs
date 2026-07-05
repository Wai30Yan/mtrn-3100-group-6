use stm32g4xx_hal::{
    gpio::{AnyPin, Output, PushPull},
    hal_02::PwmPin,
};

pub struct Motor<'a> {
    pwm: &'a mut dyn PwmPin<Duty = u16>,
    pin: AnyPin<Output<PushPull>>,
}

impl<'a> Motor<'a> {
    pub fn new(pwm: &'a mut dyn PwmPin<Duty = u16>, pin: AnyPin<Output<PushPull>>) -> Self {
        Self { pwm, pin }
    }

    /// Set the target wheel velocity in rad/s using open loop control
    pub fn set_speed(&mut self, speed: f32) {}
}
