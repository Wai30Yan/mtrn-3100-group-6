use crate::{print, state_observer};
use na::{Isometry, Isometry2};

#[derive(Clone, Copy, Debug)]
pub enum Motion {
    Idle,
    Pivot { pose: Isometry2<f32> },
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

const PIVOT_LINEAR_GAIN: f32 = 3.0;
const PIVOT_ANGULAR_GAIN: f32 = 10.0;

const LINEAR_TOLERANCE: f32 = 0.05;
const ANGULAR_TOLERANCE: f32 = 0.05;

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
            Motion::Pivot { pose } => {
                self.current_pose = current_pose;
                // Desired pose relative to the drivebase
                let pose_error = current_pose.inv_mul(&pose);

                if pose_error.translation.vector.norm() < LINEAR_TOLERANCE
                    && pose_error.rotation.angle().abs() < ANGULAR_TOLERANCE
                {
                    self.target = Motion::Idle
                }

                ChassisSpeeds {
                    vx: PIVOT_LINEAR_GAIN * pose_error.translation.x,
                    omega: PIVOT_ANGULAR_GAIN * pose_error.rotation.angle(),
                }
            }
        }
    }

    pub fn set_target(&mut self, target: Motion) {
        self.target = target;
    }
}
