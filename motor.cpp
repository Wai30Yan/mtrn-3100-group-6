#include "motor.hpp"

#include <Arduino.h>

Motor::Motor(uint8_t dir_pin, uint8_t pwm_pin, bool invert) : m_dir_pin{dir_pin}, m_pwm_pin{pwm_pin}, m_invert{invert} {
  pinMode(m_dir_pin, OUTPUT);
  pinMode(m_pwm_pin, OUTPUT);
}

void Motor::set_velocity(float v) {
  if (v > 0.0f) {
    digitalWrite(m_dir_pin, m_invert ? LOW : HIGH);
  } else {
    digitalWrite(m_dir_pin, m_invert ? HIGH : LOW);
    v = -v;
  }

  if (v < kEpsilon) {
    analogWrite(m_pwm_pin, 0);
  } else {
    analogWrite(m_pwm_pin, static_cast<uint8_t>(255 * (v * kV + kS)));
  }
}
