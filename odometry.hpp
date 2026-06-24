#pragma once

#include <Arduino.h>

struct Encoder;
struct Imu;

class Odometry {
  public:
    struct Pose {
      float x{};
      float y{};
      float theta{};
    };

    Odometry() = default;
    Odometry(const Imu& imu, const Encoder& left_encoder, const Encoder& right_encoder);

    void update();
    Pose pose() const;

  private:
    const Imu* m_imu{};
    const Encoder* m_left_encoder{};
    const Encoder* m_right_encoder{};

    Pose m_pose{};
    float m_prev_l{};
    float m_prev_r{};

    constexpr static float kWheelRadius{0.016};
    constexpr static float kWheelBase{0.108};
};
