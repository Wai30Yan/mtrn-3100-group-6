#pragma once

#include <Arduino.h>

class Motor {
  public:
    Motor() = default;
    Motor(uint8_t dir_pin, uint8_t pwm_pin, bool invert);

    // Target wheel velocity in rad/s
    void set_velocity(float v);

  private:
    uint8_t m_dir_pin;
    uint8_t m_pwm_pin;
    bool m_invert;

    constexpr static float kEpsilon{1.0f};
    constexpr static float kV{0.038f};
    constexpr static float kS{0.005f};
};
