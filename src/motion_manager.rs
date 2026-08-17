use core::matches;

use crate::na::ComplexField;
use crate::{DT, state_observer};
use na::{Isometry2, Rotation2, Translation2, UnitComplex};

#[derive(Clone, Copy, Debug, Default)]
pub enum Motion {
    #[default]
    Idle,
    // Drive in a straight line towards final_position
    Line {
        final_position: Translation2<f32>,
        final_speed: f32,
    },
    // Drive in a circular arc towards final_position
    Arc {
        final_pose: Isometry2<f32>,
        final_speed: f32,
    },
    // Rotate in place towards rotation
    Pivot {
        rotation: Rotation2<f32>,
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

#[derive(Default)]
pub struct MotionManager {
    target: Motion,
    current_pose: Isometry2<f32>,
    current_speed: f32,
}

const BASIC_ANGULAR_GAIN: f32 = 50.0;
const BASIC_LINEAR_GAIN: f32 = 10.0;
const BASIC_CROSS_GAIN: f32 = 5.0;

const MAX_VELOCITY: f32 = 0.20;
const MAX_ACCELERATION: f32 = 0.10;
const MAX_ANGULAR: f32 = 1.5;
const OVERSHOOT_GAIN: f32 = 0.2;

const GAIN_GAMMA: f32 = 0.3;
const GAIN_B: f32 = 50.0;

const EPSILON: f32 = 0.01;

impl MotionManager {
    pub fn new(pose: Isometry2<f32>) -> Self {
        Self {
            target: Default::default(),
            current_pose: pose,
            current_speed: 0.0,
        }
    }

    pub fn update(&mut self, observed_pose: Isometry2<f32>) -> ChassisSpeeds {
        match self.target {
            Motion::Idle => ChassisSpeeds::default(),
            Motion::Line {
                final_position,
                final_speed,
            } => {
                let path_delta = final_position / self.current_pose.translation;
                if path_delta.vector.norm() <= EPSILON {
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

                Self::ramsete_follow(
                    observed_pose,
                    self.current_pose,
                    ChassisSpeeds {
                        vx: self.current_speed,
                        omega: 0.0,
                    },
                )
            }
            Motion::Arc {
                final_pose,
                final_speed,
            } => {
                let path_delta = final_pose.translation / self.current_pose.translation;
                let rot_delta = final_pose.rotation / self.current_pose.rotation;
                let mut turn_rate = 0.0;

                if path_delta.vector.norm() <= EPSILON {
                    self.current_pose = final_pose;
                    self.current_speed = final_speed;
                    self.target = Motion::Idle;
                } else {
                    // Obtain the radius of rotation via the cord length
                    let rad =
                        path_delta.vector.norm() / (2.0 * f32::sin(rot_delta.angle().abs() / 2.0));

                    // Drive forward
                    self.current_speed = Self::update_speed(
                        self.current_speed,
                        final_speed,
                        rad * rot_delta.angle().abs(),
                    );
                    self.current_pose *= Translation2::new(self.current_speed * DT, 0.0);

                    // Turn
                    turn_rate = (self.current_speed / rad).copysign(rot_delta.angle());
                    self.current_pose
                        .append_rotation_wrt_center_mut(&UnitComplex::new(turn_rate * DT));
                }

                Self::ramsete_follow(
                    observed_pose,
                    self.current_pose,
                    ChassisSpeeds {
                        vx: self.current_speed,
                        omega: turn_rate,
                    },
                )
            }
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
        }
    }

    pub fn set_target(&mut self, target: Motion) {
        self.target = target;
    }

    pub fn idle(&self) -> bool {
        matches!(self.target, Motion::Idle)
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
                    + BASIC_CROSS_GAIN * error.translation.y)
                    .clamp_magnitude(MAX_ANGULAR * 0.3),
        }
    }

    // https://wiki.purduesigbots.com/software/control-algorithms/ramsete
    fn ramsete_follow(
        observed_pose: Isometry2<f32>,
        desired_pose: Isometry2<f32>,
        desired_speeds: ChassisSpeeds,
    ) -> ChassisSpeeds {
        let error = observed_pose.inv_mul(&desired_pose);
        let k = 2.0
            * GAIN_GAMMA
            * f32::hypot(desired_speeds.omega, f32::sqrt(GAIN_B) * desired_speeds.vx);
        ChassisSpeeds {
            vx: desired_speeds.vx * error.rotation.cos_angle() + k * error.translation.x,
            omega: desired_speeds.omega
                + k * error.rotation.angle()
                + GAIN_B * desired_speeds.vx * error.rotation.sin_angle() * error.translation.y
                    / error.rotation.angle(),
        }
    }
}
