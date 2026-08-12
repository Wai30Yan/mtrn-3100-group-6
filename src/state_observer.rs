use core::{
    f32,
    intrinsics::{cosf32, powf32, roundf32, sinf32},
    todo,
};

use na::{Isometry2, SMatrix, SVector, Translation2, Vector2};

use crate::{DT, encoder::Encoder, imu::Imu, print};

pub const R: f32 = 0.032;
pub const B: f32 = 0.086;
const IX: f32 = -0.004;
const IY: f32 = 0.020;

const ENCODER_COVAR: f32 = 0.002;
const IMU_COVAR: f32 = 0.0008; // * 1_000_000.0;
const HIT_WINDOW_L: f32 = 0.030;
const HIT_WINDOW_W: f32 = 0.020;

// ~10mm
const LIDAR_COVAR: f32 = 0.0001;

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
    pub fn new(pose: Isometry2<f32>) -> Self {
        Self {
            state: SVector::from_column_slice(&[
                pose.translation.x,
                pose.translation.y,
                pose.rotation.angle(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.2,
                0.0,
                0.0,
                0.0,
                0.0,
            ]),
            covar: SMatrix::from_diagonal(&SVector::from_column_slice(&[
                0.0025, 0.0025, 0.04, 0.01, 0.01, 0.01, 0.04, 0.04, 0.04,
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
            imu.gz() * 1.68,
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
                    ENCODER_COVAR,
                    ENCODER_COVAR,
                    0.01,
                    IMU_COVAR * 1_000_000.0,
                    IMU_COVAR * 1_000_000.0,
                    IMU_COVAR,
                ]));
            q.try_inverse().unwrap()
        };

        let covar = (w.transpose() * q_inv * w).try_inverse().unwrap();
        self.state = covar * w.transpose() * q_inv * y;

        self.covar = SMatrix::zeros();
        self.covar += covar.fixed_view(0, 0);
    }

    pub fn lidar_update(&mut self, distance: f32, pose: Isometry2<f32>) {
        let hit = self.pose() * pose * Translation2::new(distance, 0.0);
        // TODO: disqualify hits in certain zones (edge and obstacles)
        let wall_x = roundf32(hit.translation.x / 0.180) * 0.180;
        let wall_y = roundf32(hit.translation.y / 0.180) * 0.180;

        let (wn, wd) = if (hit.translation.x - wall_x).abs() < HIT_WINDOW_W
            && (hit.translation.y - wall_y).abs() > HIT_WINDOW_L
        {
            // Wall on x-axis
            (Vector2::new(0.0, 1.0), wall_x)
        } else if (hit.translation.y - wall_y).abs() < HIT_WINDOW_W
            && (hit.translation.x - wall_x).abs() > HIT_WINDOW_L
        {
            // Wall on y-axis
            (Vector2::new(1.0, 0.0), wall_y)
        } else {
            return;
        };

        let h = Self::h(self.state, pose, wn, wd);
        print!("{:?}\r\n", self.state);
        print!("{:?}\r\n", pose);
        print!("{:?}\r\n", wn);
        print!("{:?}\r\n", wd);
        print!("\r\n");
        let hj = Self::hj(self.state, pose, wn, wd);

        let yk = distance - h;
        let sk = (hj * self.covar * hj.transpose())[0] + LIDAR_COVAR;
        let kk_p = self.covar * hj.transpose() / sk;
        let kk = SVector::<f32, 12>::from_column_slice(&[
            kk_p[0], kk_p[1], kk_p[2], kk_p[3], kk_p[4], kk_p[5], kk_p[6], kk_p[7], kk_p[8], 0.0,
            0.0, 0.0,
        ]);
        let xk = self.state + kk * yk;
        let pk = (SMatrix::identity() - kk_p * hj) * self.covar;

        return;
        self.state = xk;
        self.covar = pk;
    }

    fn h(beta: SVector<f32, 12>, lp: Isometry2<f32>, wn: Vector2<f32>, wd: f32) -> f32 {
        f32::abs(
            -wd + wn[0]
                * (beta[0] + lp.translation.x * cosf32(beta[2])
                    - lp.translation.y * sinf32(beta[2]))
                + wn[1]
                    * (beta[1]
                        + lp.translation.y * cosf32(beta[2])
                        + lp.translation.x * sinf32(beta[2])),
        ) / f32::abs(
            wn[0] * cosf32(lp.rotation.angle() + beta[2])
                + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
        )
    }

    fn hj(
        beta: SVector<f32, 12>,
        lp: Isometry2<f32>,
        wn: Vector2<f32>,
        wd: f32,
    ) -> SMatrix<f32, 1, 9> {
        SMatrix::from_column_slice(&[
            (wn[0]
                * f32::signum(
                    wd + wn[0]
                        * (beta[0] + lp.translation.x * cosf32(beta[2])
                            - lp.translation.y * sinf32(beta[2]))
                        + wn[1]
                            * (beta[1]
                                + lp.translation.y * cosf32(beta[2])
                                + lp.translation.x * sinf32(beta[2])),
                ))
                / f32::abs(
                    wn[0] * cosf32(lp.rotation.angle() + beta[2])
                        + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
                ),
            (wn[1]
                * f32::signum(
                    wd + wn[0]
                        * (beta[0] + lp.translation.x * cosf32(beta[2])
                            - lp.translation.y * sinf32(beta[2]))
                        + wn[1]
                            * (beta[1]
                                + lp.translation.y * cosf32(beta[2])
                                + lp.translation.x * sinf32(beta[2])),
                ))
                / f32::abs(
                    wn[0] * cosf32(lp.rotation.angle() + beta[2])
                        + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
                ),
            -(f32::signum(
                wd + wn[0]
                    * (beta[0] + lp.translation.x * cosf32(beta[2])
                        - lp.translation.y * sinf32(beta[2]))
                    + wn[1]
                        * (beta[1]
                            + lp.translation.y * cosf32(beta[2])
                            + lp.translation.x * sinf32(beta[2])),
            ) * (wn[0]
                * (lp.translation.y * cosf32(beta[2]) + lp.translation.x * sinf32(beta[2]))
                - wn[1]
                    * (lp.translation.x * cosf32(beta[2]) - lp.translation.y * sinf32(beta[2]))))
                / f32::abs(
                    wn[0] * cosf32(lp.rotation.angle() + beta[2])
                        + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
                )
                - (f32::abs(
                    wd + wn[0]
                        * (beta[0] + lp.translation.x * cosf32(beta[2])
                            - lp.translation.y * sinf32(beta[2]))
                        + wn[1]
                            * (beta[1]
                                + lp.translation.y * cosf32(beta[2])
                                + lp.translation.x * sinf32(beta[2])),
                ) * f32::signum(
                    wn[0] * cosf32(lp.rotation.angle() + beta[2])
                        + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
                ) * (wn[1] * cosf32(lp.rotation.angle() + beta[2])
                    - wn[0] * sinf32(lp.rotation.angle() + beta[2])))
                    / powf32(
                        f32::abs(
                            wn[0] * cosf32(lp.rotation.angle() + beta[2])
                                + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
                        ),
                        2.0,
                    ),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ])
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
