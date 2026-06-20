#include <Arduino.h>
#include "motor.hpp"
#include "encoder.hpp"

// right motor PWM = 9, DIR = 10 - MOTOR 2
// left motor PWM = 11, DIR = 12 - MOTOR 1
#define RIGHT_PWM 9
#define RIGHT_DIR 10
#define LEFT_PWM 11
#define LEFT_DIR 12

#define RIGHT_ENA 3
#define RIGHT_ENB 8
#define LEFT_ENA 2
#define LEFT_ENB 7

#define LEFT_LIDER A0
#define RIGHT_LIDER A1
#define FRONT_LIDER A3

const float WHEEL_DIAMETER_MM = 32.0;
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0;

Motor rightMotor(RIGHT_DIR, RIGHT_PWM);
Motor leftMotor(LEFT_DIR, LEFT_PWM);

Encoder rightEncoder(RIGHT_ENA, RIGHT_ENB);
Encoder leftEncoder(LEFT_ENA, LEFT_ENB);

void setup() {
    Serial.begin(9600);
}

void loop() {
    rightMotor.spin(255, true);

    float radsNow = rightEncoder.position();
    long countsNow = rightEncoder.count;

    float distanceMM = radsNow * WHEEL_RADIUS_MM;

    if (distanceMM >= 200) {
        Serial.println("Stopping at ");
        Serial.print(distanceMM);
        Serial.println(" mm");
        while(1);
    } else {
        Serial.print("Raw Count: ");
        Serial.print(countsNow);
        Serial.print("  |  Angle: ");
        Serial.print(radsNow);
        Serial.print(" rad  |  Distance: ");
        Serial.print(distanceMM);
        Serial.println(" mm");
    }
    
    delay(100);
}
