use core::intrinsics::{black_box, breakpoint};
use core::todo;

use crate::na::ComplexField;
use crate::{DT, print, state_observer};
use na::{Isometry, Isometry2, Rotation2, Translation2};

#[derive(Clone, Copy, Debug)]
pub enum Motion {
    Idle,
    Pivot {
        rotation: Rotation2<f32>,
    },
    Pose {
        pose: Isometry2<f32>,
    },
    Line {
        final_position: Translation2<f32>,
        final_speed: f32,
    },
    Arc {
        centre: Translation2<f32>,
        radius: f32,
        final_speed: f32,
    },
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ChassisSpeeds {
    vx: f32,
    omega: f32,
}

impl ChassisSpeeds {
    pub fn to_wheel_velocities(&self) -> (f32, f32) {
        (
            1.0 / state_observer::R * (2.0 * self.vx - state_observer::B * self.omega),
            1.0 / state_observer::R * (2.0 * self.vx + state_observer::B * self.omega),
        )
    }
}

pub struct MotionManager {
    target: Motion,
    current_pose: Isometry2<f32>,
    current_speed: f32,
}

const BASIC_ANGULAR_GAIN: f32 = 50.0;
const BASIC_LINEAR_GAIN: f32 = 10.0;

const MAX_VELOCITY: f32 = 0.20;
const MAX_ACCELERATION: f32 = 0.10;
const MAX_ANGULAR: f32 = 2.0;
const OVERSHOOT_GAIN: f32 = 0.2;

const GAMMA_GAIN: f32 = 0.3;
const B_GAIN: f32 = 20.0;

const LINEAR_TOLERANCE: f32 = 0.03;
const ANGULAR_TOLERANCE: f32 = 0.03;
const EPSILON: f32 = 0.005;

impl MotionManager {
    pub fn new() -> Self {
        Self {
            target: Motion::Idle,
            current_pose: Isometry::identity(),
            current_speed: 0.0,
        }
    }

    pub fn update(&mut self, observed_pose: Isometry2<f32>) -> ChassisSpeeds {
        match self.target {
            Motion::Idle => ChassisSpeeds::default(),
            Motion::Pivot { rotation } => {
                self.current_pose.rotation = rotation.into();
                self.current_speed = 0.0;

                let error = (rotation / observed_pose.rotation).angle();

                if error.abs() <= EPSILON {
                    self.target = Motion::Idle;
                }

                ChassisSpeeds {
                    vx: 0.0,
                    omega: (BASIC_ANGULAR_GAIN * error).clamp_magnitude(MAX_ANGULAR),
                }
            }
            Motion::Pose { pose } => {
                self.current_pose = pose;
                self.current_speed = 0.0;

                let error = observed_pose.inv_mul(&pose);

                if error.translation.vector.norm() <= LINEAR_TOLERANCE
                    && error.rotation.angle().abs() <= ANGULAR_TOLERANCE
                {
                    self.target = Motion::Idle;
                }

                ChassisSpeeds {
                    vx: (BASIC_LINEAR_GAIN * error.translation.x).clamp_magnitude(MAX_VELOCITY),
                    omega: (BASIC_ANGULAR_GAIN * error.rotation.angle())
                        .clamp_magnitude(MAX_ANGULAR),
                }
            }
            Motion::Line {
                final_position,
                final_speed,
            } => {
                let path_delta = final_position / self.current_pose.translation;
                if path_delta.vector.norm() <= LINEAR_TOLERANCE {
                    self.current_pose.translation = final_position;
                    self.current_speed = final_speed;
                    self.target = Motion::Idle;
                } else {
                    self.current_speed = Self::update_speed(
                        self.current_speed,
                        final_speed,
                        path_delta.vector.norm(),
                    );
                    // Move forward at the desired speed
                    self.current_pose *= Translation2::new(self.current_speed * DT, 0.0);
                }

                Self::dumb_follow(
                    observed_pose,
                    self.current_pose,
                    ChassisSpeeds {
                        vx: self.current_speed,
                        omega: 0.0,
                    },
                )
            }
            Motion::Arc {
                centre,
                radius,
                final_speed,
            } => todo!(),
        }
    }

    pub fn set_target(&mut self, target: Motion) {
        self.target = target;
    }

    pub fn idle(&self) -> bool {
        if let Motion::Idle = self.target {
            true
        } else {
            false
        }
    }

    pub fn pose(&self) -> Isometry2<f32> {
        self.current_pose
    }

    fn update_speed(current_speed: f32, final_speed: f32, remaining_distance: f32) -> f32 {
        // Multiplying MAX_ACCELERATION by DT instead of DT^2 fudges the numbers
        // slightly to prevent overshoot
        if remaining_distance <= final_speed * DT + MAX_ACCELERATION * DT {
            return final_speed;
        }

        let stopping_distance =
            (current_speed * current_speed - final_speed * final_speed) / (2.0 * MAX_ACCELERATION);

        if remaining_distance <= stopping_distance {
            current_speed
                - MAX_ACCELERATION * DT
                - (stopping_distance - remaining_distance) * OVERSHOOT_GAIN
        } else {
            (current_speed + MAX_ACCELERATION * DT).clamp_magnitude(MAX_VELOCITY)
        }
    }

    fn ramsete(
        observed_pose: Isometry2<f32>,
        desired_pose: Isometry2<f32>,
        desired_speeds: ChassisSpeeds,
    ) -> ChassisSpeeds {
        let error = observed_pose.inv_mul(&desired_pose);
        let k =
            2.0 * GAMMA_GAIN * f32::hypot(desired_speeds.omega, B_GAIN.sqrt() * desired_speeds.vx);

        ChassisSpeeds {
            vx: (desired_speeds.vx * error.rotation.cos_angle() + k * error.translation.x)
                .clamp_magnitude(MAX_VELOCITY * 1.2),
            omega: (desired_speeds.omega
                + k * error.rotation.angle()
                + (B_GAIN * desired_speeds.vx * error.rotation.sin_angle() * error.translation.y)
                    / error.rotation.angle())
            .clamp_magnitude(MAX_ANGULAR * 1.2),
        }
    }

    fn dumb_follow(
        observed_pose: Isometry2<f32>,
        desired_pose: Isometry2<f32>,
        desired_speeds: ChassisSpeeds,
    ) -> ChassisSpeeds {
        let error = observed_pose.inv_mul(&desired_pose);
        ChassisSpeeds {
            vx: desired_speeds.vx
                + (BASIC_LINEAR_GAIN * error.translation.x).clamp_magnitude(MAX_VELOCITY * 0.2),
            omega: desired_speeds.omega
                + (BASIC_ANGULAR_GAIN * error.rotation.angle()
                    + BASIC_LINEAR_GAIN * error.translation.y)
                    .clamp_magnitude(MAX_ANGULAR * 0.2),
        }
    }
}
