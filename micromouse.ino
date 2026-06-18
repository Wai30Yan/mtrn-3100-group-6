#include <Arduino.h>
#include "motor.hpp"

// right motor PWM = 9, DIR = 10 - MOTOR 2
// left motor PWM = 11, DIR = 12 - MOTOR 1
#define RIGHT_PWM 9
#define RIGHT_DIR 10
#define LEFT_PWM 11
#define LEFT_DIR 12

#define RIGHT_ENA D3
#define RIGHT_ENB D8
#define LEFT_ENA D2
#define LEFT_ENB D7

#define LEFT_LIDER A0
#define RIGHT_LIDER A1
#define FRONT_LIDER A3

Motor rightMotor(RIGHT_DIR, RIGHT_PWM);
Motor leftMotor(LEFT_DIR, LEFT_PWM);

void setup() {
    Serial.begin(9600);

    rightMotor.begin();
    leftMotor.begin();
}

void loop() {

}

void go_forward() {
    rightMotor.drive(HIGH, 130);
    leftMotor.drive(HIGH, 130);
    delay(3000);
}

void go_left() {
    rightMotor.drive(HIGH, 130);
    leftMotor.drive(LOW, 130);
    delay(3000);
}

void stop() {
    rightMotor.brake();
    leftMotor.brake();
    delay(3000);
}