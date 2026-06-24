#pragma once

#include <Arduino.h>

class Encoder {
  public:
    Encoder() = default;
    Encoder(bool channel, bool invert);

    // Get the current position of the encoder in radians
    float position() const;
    void callback(uint8_t state);

  private:
    bool m_channel;
    bool m_invert;

    constexpr static float kScale{6.282f / 1400.0f};
};
