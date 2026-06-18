#include "motor.hpp"

Motor::Motor(uint8_t dirPin, uint8_t pwmPin) {
    this->dirPin = dirPin;
    this->pwmPin = pwmPin;
    this->targetVelocity = 0.0;

    pinMode(this->dirPin, OUTPUT);
    pinMode(this->pwmPin, OUTPUT);

    digitalWrite(this->dirPin, LOW);
    analogWrite(this->pwmPin, 0);
}

void Motor::set_velocity(float v) {
    targetVelocity = v;
}

void Motor::drive(uint8_t dirValue, int pwmValue) {
    digitalWrite(dirPin, dirValue);
    analogWrite(pwmPin, pwmValue);
}

void Motor::brake() {
    analogWrite(pwmPin, 0);
}
