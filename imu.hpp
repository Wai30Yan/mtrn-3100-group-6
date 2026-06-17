#pragma once

#include <Arduino.h>
#include <Wire.h>

class Imu {
  public:
    Imu() = default;
    Imu(TwoWire& w);

    void update();

    // Linear acceleration in the x direction in m/s^2
    // fwd = +ve, rev = -ve
    float ax() const;

    // Linear acceleration in the y direction in m/s^2
    // left = +ve, right = -ve
    float ay() const;

    // Angular velocity about the z axis in rad/s
    // ccw = +ve, cw = -ve
    float gz() const;

  private:
    void write_reg(uint8_t addr, uint8_t data);
    uint8_t read_reg(uint8_t addr);

    TwoWire* w;
    float m_ax{0.0};
    float m_ay{0.0};
    float m_gz{0.0};

    constexpr static uint8_t kAddr{0x68};
    constexpr static uint8_t kPwrMgmt1Reg{0x6b};
    constexpr static uint8_t kSmplrtDivReg{0x19};
    constexpr static uint8_t kConfigReg{0x1a};
    constexpr static uint8_t kGyroConfigReg{0x1b};
    constexpr static uint8_t kAccelConfigReg{0x1c};
    constexpr static uint8_t kAccelOutReg{0x3B};

    constexpr static float kAccFactor{9.81f/8192.0f};
    constexpr static float kGyroFactor{1.0f/1.143191f};
};
