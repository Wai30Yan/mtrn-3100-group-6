use na::{Isometry2, SVector, Vector2};

use crate::{encoder::Encoder, imu::Imu};

pub struct StateObserver {
    // The state vector contains the translation, velocities, linear
    // accelerations and IMU biases
    // tx, ty, tt,
    // vx, vy, vt
    // bx, by, bt,
    // ax, ay, at (redundant)
    state: SVector<f32, 12>,
}

impl StateObserver {
    pub fn new() -> Self {
        Self {
            state: SVector::zeros(),
        }
    }

    pub fn update(&mut self, imu: &Imu, encoder_left: &dyn Encoder, encoder_right: &dyn Encoder) {
        /* 
         * We are not using the standard formula of the Kalman Filter for the
         * update step as we do not have a model of the system just
         * relationships between the state and the sensor values.
         * 
         * Based off of https://arxiv.org/pdf/1910.03558 (pg. 11).
         * 
         * We perform the calculations as though y is a vector with 15 elements
         * (from the nine previous states, five measurements and one assumption
         * of minimal lateral slip). The \beta vector corresponds to the new
         * state with three redundant states added.
         * 
         * The true model is z = f(\beta) + \epislon this can be linearised to
         * z = W\beta + \alpha + \epislon where W is the Jacobian and
         * \alpha = f(\beta) - W\beta. To convert into the required form, let
         * y = z - \alpha. This roughly corresponds to y being the measurement
         * residuals. This linearisation is valid is we assume that the state
         * will change slowly over time and f is relatively well behaved.
         * 
         * There are three more observations than there are states so it is
         * high unlikely that the system will be underdefined.
         */
    }

    pub fn pose(&self) -> Isometry2<f32> {
        Isometry2::new(Vector2::new(self.state[0], self.state[1]), self.state[2])
    }
}
