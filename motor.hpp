#pragma once

#include <Arduino.h>

class Motor {
  public:
    // TODO: add required args
    Motor(uint8_t dirPin, uint8_t pwnPin);

    // Target wheel velocity in rad/s
    void set_velocity(float v);

    void update_speed(float measuredVelocity);

    // set values for DIR and PWN values
    // DIR for direction of the spin, PWM for speed of the spin
    // HIGH - LOW for DIR; 0-255 for PWM
    void drive(uint8_t dirValue, int pwmValue);

    // set PWM to zero to stop spinning
    void brake();

  private:
    // TODO: add member variables
    uint8_t dirPin;
    uint8_t pwmPin;
    float targetVelocity;  // rad/s
};
