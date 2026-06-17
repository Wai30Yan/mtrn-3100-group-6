#include "imu.hpp"

Imu::Imu(TwoWire& w) : w{&w} {
  write_reg(kPwrMgmt1Reg, 0x01);
  write_reg(kSmplrtDivReg, 0x00);
  write_reg(kConfigReg, 0x00);
  // Second lowest range for both
  write_reg(kGyroConfigReg, 0x08);
  write_reg(kAccelConfigReg, 0x08);
}

float to_float(uint16_t x) {
  return static_cast<float>(static_cast<int16_t>(
    ((x & 0x00FF) << 8) +
    ((x & 0xFF00) >> 8)
  ));
}

void Imu::update() {
  w->beginTransmission(kAddr);
  w->write(kAccelOutReg);
  w->endTransmission(false);
  w->requestFrom(kAddr, static_cast<uint8_t>(14));

  uint16_t raw_data[7]; // [ax,ay,az,temp,gx,gy,gz]
  w->readBytes(reinterpret_cast<char*>(raw_data), 14);

  m_ax = static_cast<float>(to_float(raw_data[0])) * kAccFactor;
  m_ay = static_cast<float>(to_float(raw_data[1])) * kAccFactor;
  m_gz = static_cast<float>(to_float(raw_data[6])) * kGyroFactor;
}

float Imu::ax() const {
  return m_ax;
}

float Imu::ay() const {
  return m_ay;
}

float Imu::gz() const {
  return m_gz;
}

void Imu::write_reg(uint8_t reg, uint8_t data) {
  w->beginTransmission(kAddr);
  w->write(reg);
  w->write(data);
  w->endTransmission();
}

uint8_t Imu::read_reg(uint8_t reg) {
  w->beginTransmission(kAddr);
  w->write(reg);
  w->endTransmission(true);

  w->requestFrom(kAddr, static_cast<uint8_t>(1));
  return w->read();
}
