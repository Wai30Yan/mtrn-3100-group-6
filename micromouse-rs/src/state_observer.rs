use core::intrinsics::{cosf32, sinf32};

use cortex_m::asm::delay;
use na::{Isometry2, SMatrix, SVector, Vector2};

use crate::{DT, encoder::Encoder, imu::Imu, print};

const R: f32 = 0.032;
const B: f32 = 0.086;
const IX: f32 = 0.011;
const IY: f32 = 0.020;

pub struct StateObserver {
    // The state vector contains the translation, velocities, linear
    // accelerations and IMU biases
    // tx, ty, tt,
    // vx, vy, vt
    // bx, by, bt,
    // ax, ay, at (redundant)
    state: SVector<f32, 12>,
    covar: SMatrix<f32, 9, 9>,

    prev_left: f32,
    prev_right: f32,
}

impl StateObserver {
    pub fn new() -> Self {
        Self {
            state: SVector::zeros(),
            covar: SMatrix::from_diagonal(&SVector::from_column_slice(&[
                0.0025, 0.0025, 0.04, 0.0001, 0.0001, 0.01, 0.04, 0.04, 0.04,
            ])),

            prev_left: 0.0,
            prev_right: 0.0,
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

        let z = SVector::from_column_slice(&[
            self.state[0],
            self.state[1],
            self.state[2],
            self.state[3],
            self.state[4],
            self.state[5],
            self.state[6],
            self.state[7],
            self.state[8],
            encoder_left.position() - self.prev_left,
            encoder_right.position() - self.prev_right,
            0.0,
            imu.ax(),
            imu.ay(),
            imu.gz(),
        ]);

        self.prev_left = encoder_left.position();
        self.prev_right = encoder_right.position();

        let w = Self::w(self.state);
        let alpha = Self::f(self.state) - w * self.state;
        let y = z - alpha;

        let q_inv = {
            let mut q = SMatrix::<f32, 15, 15>::zeros();

            let mut upper = q.fixed_view_mut::<9, 9>(0, 0);
            upper += self.covar;

            // TODO: set correct covars
            q.fixed_view_mut::<6, 6>(9, 9)
                .set_diagonal(&SVector::from_column_slice(&[
                    0.001, 0.001, 0.01, 0.001, 0.001, 0.001,
                ]));
            q.try_inverse().unwrap()
        };

        let covar = (w.transpose() * q_inv * w).try_inverse().unwrap();
        self.state = covar * w.transpose() * q_inv * y;

        self.covar = SMatrix::zeros();
        self.covar += covar.fixed_view(0, 0);
    }

    fn f(beta: SVector<f32, 12>) -> SVector<f32, 15> {
        SVector::from_column_slice(&[
            (beta[9] * DT * DT) / 2. - beta[3] * DT + beta[0],
            (beta[10] * DT * DT) / 2. - beta[4] * DT + beta[1],
            (beta[11] * DT * DT) / 2. - beta[5] * DT + beta[2],
            beta[3] - beta[9] * DT,
            beta[4] - beta[10] * DT,
            beta[5] - beta[11] * DT,
            beta[6],
            beta[7],
            beta[8],
            (DT * (2.
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[3] - (beta[9] * DT) / 2.)
                + 2. * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.)
                - B * (beta[5] - (beta[11] * DT) / 2.)))
                / R,
            (DT * (2.
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[3] - (beta[9] * DT) / 2.)
                + 2. * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.)
                + B * (beta[5] - (beta[11] * DT) / 2.)))
                / R,
            cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[4] - (beta[10] * DT) / 2.)
                - sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.),
            beta[6]
                + beta[11] * IY
                + IX * beta[5] * beta[5]
                + beta[9] * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                + beta[10] * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            beta[7]
                + beta[11] * IX
                + IY * beta[5] * beta[5]
                + beta[10] * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                - beta[9] * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            beta[8] + beta[5],
        ])
    }

    fn w(beta: SVector<f32, 12>) -> SMatrix<f32, 15, 12> {
        SMatrix::from_row_slice(&[
            1.,
            0.,
            0.,
            -DT,
            0.,
            0.,
            0.,
            0.,
            0.,
            DT * DT / 2.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            -DT,
            0.,
            0.,
            0.,
            0.,
            0.,
            DT * DT / 2.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            -DT,
            0.,
            0.,
            0.,
            0.,
            0.,
            DT * DT / 2.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            0.,
            -DT,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            0.,
            -DT,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            0.,
            -DT,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
            0.,
            0.,
            (DT * (2.
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[4] - (beta[10] * DT) / 2.)
                - 2. * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.)))
                / R,
            (2. * DT * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            (2. * DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            -(DT * (B + DT
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[4] - (beta[10] * DT) / 2.)
                - DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.)))
                / R,
            0.,
            0.,
            0.,
            -(DT * DT * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            -(DT * DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            (DT * ((B * DT) / 2.
                + (DT
                    * DT
                    * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.))
                    / 2.
                - (DT
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.))
                    / 2.))
                / R,
            0.,
            0.,
            (DT * (2.
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[4] - (beta[10] * DT) / 2.)
                - 2. * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.)))
                / R,
            (2. * DT * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            (2. * DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            (DT * (B - DT
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[4] - (beta[10] * DT) / 2.)
                + DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.)))
                / R,
            0.,
            0.,
            0.,
            -(DT * DT * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            -(DT * DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / R,
            -(DT * ((B * DT) / 2.
                - (DT
                    * DT
                    * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.))
                    / 2.
                + (DT
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[3] - (beta[9] * DT) / 2.))
                    / 2.))
                / R,
            0.,
            0.,
            -cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[3] - (beta[9] * DT) / 2.)
                - sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.),
            -sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            (DT * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[3] - (beta[9] * DT) / 2.))
                / 2.
                + (DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.))
                    / 2.,
            0.,
            0.,
            0.,
            (DT * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / 2.,
            -(DT * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])) / 2.,
            -(DT * DT
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                * (beta[3] - (beta[9] * DT) / 2.))
                / 4.
                - (DT
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                    * (beta[4] - (beta[10] * DT) / 2.))
                    / 4.,
            0.,
            0.,
            beta[10] * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                - beta[9] * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            0.,
            0.,
            2. * IX * beta[5]
                - (beta[10]
                    * DT
                    * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                    / 2.
                + (beta[9]
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                    / 2.,
            1.,
            0.,
            0.,
            cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            IY + (beta[10]
                * DT
                * DT
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                / 4.
                - (beta[9]
                    * DT
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                    / 4.,
            0.,
            0.,
            -beta[9] * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2])
                - beta[10] * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            0.,
            0.,
            2. * IY * beta[5]
                + (beta[9]
                    * DT
                    * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                    / 2.
                + (beta[10]
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                    / 2.,
            0.,
            1.,
            0.,
            -sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]),
            IX - (beta[9]
                * DT
                * DT
                * cosf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                / 4.
                - (beta[10]
                    * DT
                    * DT
                    * sinf32((beta[11] * DT * DT) / 4. - (beta[5] * DT) / 2. + beta[2]))
                    / 4.,
            0.,
            0.,
            0.,
            0.,
            0.,
            1.,
            0.,
            0.,
            1.,
            0.,
            0.,
            0.,
        ])
    }

    pub fn pose(&self) -> Isometry2<f32> {
        Isometry2::new(Vector2::new(self.state[0], self.state[1]), self.state[2])
    }
}
