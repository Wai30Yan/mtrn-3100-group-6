#pragma once

class Imu {
  public:
    // TODO: add required args
    Imu();

    // Linear acceleration in the x direction in m/s^2
    // fwd = +ve, rev = -ve
    float ax();

    // Linear acceleration in the y direction in m/s^2
    // left = +ve, right = -ve
    float ay();

    // Angular velocity about the z axis in rad/s
    // ccw = +ve, cw = -ve
    float gz();

  private:
    // TODO: add member variables
};
