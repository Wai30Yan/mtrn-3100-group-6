#pragma once
#include <Arduino.h>

class Encoder {
  public:
    // TODO: add required args
    Encoder(uint8_t a, uint8_t b);
    void readEncoder();

    // Get the current position of the encoder in radians
    // float position() const;
    float getRotation();

  private:
    static void readEncoderISR();

  public:
    // TODO: add member variables
    const uint8_t pinA, pinB;
    volatile int8_t direction;
    volatile long count = 0;
    float current_position_rad = 0;
    // 2800 for Interrupt mode CHANGE
    // 1400 for Interrupt mode RISE/FALL
    uint16_t counts_per_revolution = 2800;
    uint32_t prev_time;
    bool read = false;

  private:
    static Encoder* instance;
};
