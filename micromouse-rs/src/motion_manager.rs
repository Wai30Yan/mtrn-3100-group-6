use crate::{na::ComplexField, print, state_observer};
use na::{Complex, Isometry, Isometry2, Normed, Translation2, UnitComplex, Vector2};

use crate::DT;

#[derive(Clone, Copy, Debug)]
pub enum Motion {
    Path { pose: Isometry2<f32>, speed: f32 },
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
    current_speed: f32,
}

const PIVOT_LINEAR_GAIN: f32 = 10.0;
const PIVOT_ANGULAR_GAIN: f32 = 10.0;

const OVERSHOOT_GAIN: f32 = 0.2;
const GAMMA_GAIN: f32 = 0.2;
const B_GAIN: f32 = 1.0;
const TURN_GAIN: f32 = 0.1;

const MAX_VELOCITY: f32 = 1.0;
const MAX_ACCELERATION: f32 = 1.0;

impl MotionManager {
    pub fn new() -> Self {
        Self {
            target: Motion::Pivot {
                pose: Isometry::identity(),
            },
            current_pose: Isometry::identity(),
            current_speed: 0.0,
        }
    }

    pub fn update(&mut self, current_pose: Isometry2<f32>) -> ChassisSpeeds {
        match self.target {
            Motion::Path { pose, speed } => {
                let ti = self.current_pose.translation.vector;
                let vi = self.current_speed
                    * self
                        .current_pose
                        .rotation
                        .transform_vector(&Vector2::new(1.0, 0.0));

                let tf = pose.translation.vector;
                let vf = speed * pose.rotation.transform_vector(&Vector2::new(1.0, 0.0));

                // print!("{:?}\r\n", self.current_pose);

                let (x, vx) = Self::lspb_step(ti.x, vi.x, tf.x, vf.x);
                let (y, vy) = Self::lspb_step(ti.y, vi.y, tf.y, vf.y);

                // We are done so pivot about target
                if x == tf.x && y == tf.y && vx == vf.x && vy == vf.y {
                    self.target = Motion::Pivot { pose }
                }

                let velocity = Complex::new(vx, vy);
                self.current_pose = Isometry2::from_parts(
                    Translation2::new(x, y),
                    UnitComplex::from_complex(velocity),
                );
                // Desired pose relative to the drivebase
                let pose_error = current_pose.inv_mul(&pose);

                let vd = velocity.norm();
                self.current_speed = vd;
                let wd = TURN_GAIN * pose_error.rotation.angle();

                let k = 2.0 * GAMMA_GAIN * f32::hypot(wd, B_GAIN * vd);

                ChassisSpeeds {
                    vx: vd * pose_error.rotation.cos_angle() + k * pose_error.translation.x,
                    omega: wd
                        + k * pose_error.rotation.angle()
                        + B_GAIN * vd * pose_error.rotation.sin_angle() * pose_error.translation.y
                            / pose_error.rotation.angle(),
                }
            }
            Motion::Pivot { pose } => {
                self.current_pose = current_pose;
                // Desired pose relative to the drivebase
                let pose_error = current_pose.inv_mul(&pose);

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

    fn lspb_step(qi: f32, vi: f32, qf: f32, vf: f32) -> (f32, f32) {
        let target_dist = f32::abs(qi - qf);
        // Multiplying MAX_ACCELERATION by DT instead of DT^2 fudges the numbers
        // slightly to prevent overshoot
        if target_dist < f32::abs(vf * DT) + MAX_ACCELERATION * DT
            && f32::abs(vi - vf) <= MAX_ACCELERATION * DT
        {
            // We make it to the final state in this tick, force to exactly the state
            return (qf, vf);
        }

        // Within the stopping distance
        let stopping_distance = f32::abs(vf * vf - vi * vi) / (2.0 * MAX_ACCELERATION);
        if target_dist <= stopping_distance {
            // Copysign to approach the desired velocity
            return (
                qi + vi * DT,
                vi + (MAX_ACCELERATION * DT + (stopping_distance - target_dist) * OVERSHOOT_GAIN)
                    .copysign(vf - vi),
            );
        }

        // Copysign to accelerate towards the target
        return (
            qi + vi * DT,
            (vi + (MAX_ACCELERATION * DT).copysign(qf - qi)).clamp_magnitude(MAX_VELOCITY),
        );
    }
}
