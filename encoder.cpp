#include "encoder.hpp"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder(uint8_t a, uint8_t b) : pinA(a), pinB(b) {
  instance = this;
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA), readEncoderISR, CHANGE);
}

void Encoder::readEncoder() {
  noInterrupts();

  if (digitalRead(pinB) == HIGH) {
    count++;
    direction = 1;
  } else {
    count--;
    direction = -1;
  }

  interrupts();
}

float Encoder::getRotation() {
  if (counts_per_revolution == 0) return 0.0;

  noInterrupts();
  long currentCount = count;
  interrupts();

  return ((float)currentCount / counts_per_revolution) * 2.0 * 3.14159;
}

// float Encoder::position() const {
//   return const_cast<Encoder*>(this)->getRotation();
// }

void Encoder::readEncoderISR() {
  if (instance != nullptr) instance->readEncoder();
}