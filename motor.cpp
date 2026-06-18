#include "motor.hpp"

Motor::Motor(uint8_t dirPin, uint8_t pwmPin) {
    this->dirPin = dirPin;
    this->pwmPin = pwmPin;
    this->targetVelocity = 0.0;
}

void Motor::set_velocity(float v) {
    targetVelocity = v;
}

void Motor::update_speed(float measuredVelocity) {
    if (targetVelocity == 0.0) {
        brake();
        return;
    }

    float error = targetVelocity - measuredVelocity;

    float controlSignal = error * Kp;

    uint8_t direction = (targetVelocity >= 0) ? HIGH : LOW;
    int pwmOutput = abs((int)controlSignal);

    drive(direction, pwmOutput);
}

void Motor::begin() {
    pinMode(dirPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    brake();
}

void Motor::drive(uint8_t dirValue, uint8_t pwmValue) {
    digitalWrite(dirPin, dirValue);
    analogWrite(pwmPin, pwmValue);
}

void Motor::brake() {
    analogWrite(pwmPin, 0);
}
