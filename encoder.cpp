#include "encoder.hpp"

#include <Arduino.h>

struct EncoderState {
  volatile int32_t m_count{0};

  void callback(bool a, bool b) {
    if (a ^ b) {
      m_count++;
    } else {
      m_count--;
    }
  }
};

EncoderState encoder_a{};
EncoderState encoder_b{};

static void interrupt_a() {
  encoder_a.callback(digitalRead(2), digitalRead(7));
}

static void interrupt_b() {
  encoder_b.callback(digitalRead(3), digitalRead(8));
}

Encoder::Encoder(bool channel, bool invert) : m_channel{channel}, m_invert{invert} {
  if (!channel) {
    pinMode(2, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(2), interrupt_a, CHANGE);
  } else {
    pinMode(3, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(3), interrupt_b, CHANGE);
  }
}

float Encoder::position() const {
  int32_t count = (!m_channel ? encoder_a : encoder_b).m_count;
  return static_cast<float>(count) * kScale * (m_invert ? -1.0f : 1.0f);
}
