#include "odometry.hpp"

#include "encoder.hpp"

Odometry::Odometry(const Imu& imu, const Encoder& left_encoder, const Encoder& right_encoder) :
  m_imu{&imu}, m_left_encoder{&left_encoder}, m_right_encoder{&right_encoder} {
    m_prev_l = m_left_encoder->position();
    m_prev_r = m_right_encoder->position();
}

void Odometry::update() {
  float l = m_left_encoder->position() * kWheelRadius;
  float r = m_right_encoder->position() * kWheelRadius;
  float l_dist = l - m_prev_l;
  float r_dist = r - m_prev_r;

  float dd = (l_dist + r_dist) / 2.0f;
  float dth = (r_dist - l_dist) / kWheelBase;

  m_pose.x += dd * cos(m_pose.theta);
  m_pose.y += dd * sin(m_pose.theta);
  m_pose.theta += dth;

  m_prev_l = l;
  m_prev_r = r;
}

Odometry::Pose Odometry::pose() const {
  return m_pose;
}
