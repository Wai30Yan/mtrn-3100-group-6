#include <Wire.h>

#include "imu.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "odometry.hpp"

Imu imu;

Motor left_motor;
Motor right_motor;

Encoder left_encoder;
Encoder right_encoder;

Odometry odometry;

void setup() {
  delay(1000);
  //Serial.begin(115200);
  imu = Imu(Wire);

  left_motor = Motor(12, 11, true);
  right_motor = Motor(10, 9, false);

  left_encoder = Encoder(false, true);
  right_encoder = Encoder(true, false);

  odometry = Odometry(imu, left_encoder, right_encoder);

  while (odometry.pose().x < 0.1) {
    odometry.update();
    auto pose = odometry.pose();
    auto delta = pose.theta * 100.0f;
    left_motor.set_velocity(20 + delta);
    right_motor.set_velocity(20 - delta);
  }

  for (int j = 0; j < 100; j++) {
    odometry.update();
    delay(3);
  }

  for (int i = 1; i < 5; i++) {
    while (odometry.pose().theta < i*1.65f) {
      odometry.update();
      left_motor.set_velocity(-10);
      right_motor.set_velocity(10);
    }
    left_motor.set_velocity(0);
    right_motor.set_velocity(0);
    for (int j = 0; j < 100; j++) {
      odometry.update();
      delay(3);
    }
  }

  for (int i = 1; i < 5; i++) {
    while (odometry.pose().theta > 6.60f - i*1.65f) {
      odometry.update();
      left_motor.set_velocity(10);
      right_motor.set_velocity(-10);
    }
    left_motor.set_velocity(0);
    right_motor.set_velocity(0);
    for (int j = 0; j < 100; j++) {
      odometry.update();
      delay(3);
    }
  }
}

void loop() {
}
