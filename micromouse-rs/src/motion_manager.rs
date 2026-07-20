use crate::{print, state_observer};
use na::{Isometry, Isometry2};

#[derive(Clone, Copy, Debug)]
pub enum Motion {
    Idle,
    Pivot {
        pose: Isometry2<f32>,
        ignore_translation: bool,
    },
}

#[derive(Clone, Copy, Debug)]
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
}

const PIVOT_LINEAR_GAIN: f32 = 5.0;
const PIVOT_ANGULAR_GAIN: f32 = 20.0;

impl MotionManager {
    pub fn new() -> Self {
        Self {
            target: Motion::Idle,
            current_pose: Isometry::identity(),
        }
    }

    pub fn update(&mut self, current_pose: Isometry2<f32>) -> ChassisSpeeds {
        match self.target {
            Motion::Idle => ChassisSpeeds {
                vx: 0.0,
                omega: 0.0,
            },
            Motion::Pivot {
                pose,
                ignore_translation,
            } => {
                self.current_pose = current_pose;
                // Desired pose relative to the drivebase
                let pose_error = current_pose.inv_mul(&pose);

                ChassisSpeeds {
                    vx: if ignore_translation {
                        0.0
                    } else {
                        PIVOT_LINEAR_GAIN * pose_error.translation.x
                    },
                    omega: PIVOT_ANGULAR_GAIN * pose_error.rotation.angle(),
                }
            }
        }
    }

    pub fn set_target(&mut self, target: Motion) {
        self.target = target;
    }
}
