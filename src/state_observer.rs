use core::{
    f32,
    intrinsics::{cosf32, powf32, roundf32, sinf32},
};

use na::{Isometry2, SMatrix, SVector, Translation2, Vector2, Vector3};

use crate::{CELL_SIZE, encoder::Encoder, print};

pub const R: f32 = 0.032;
pub const B: f32 = 0.083;

const ENCODER_COVAR: f32 = 0.002;
const HIT_WINDOW_L: f32 = 0.030;
const HIT_WINDOW_W: f32 = 0.030;
const MAX_HIT_ANGLE: f32 = f32::consts::FRAC_PI_4;

// ~10mm
const LIDAR_COVAR: f32 = 0.0001;

pub struct StateObserver {
    // tx, ty, tt,
    state: SVector<f32, 3>,
    covar: SMatrix<f32, 3, 3>,

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
            ]),
            covar: SMatrix::from_diagonal(&Vector3::new(0.0025, 0.0025, 0.04)),

            prev_left: 0.0,
            prev_right: 0.0,
        }
    }

    pub fn update(&mut self, encoder_left: &dyn Encoder, encoder_right: &dyn Encoder) {
        let dwl = encoder_left.position() - self.prev_left;
        let dwr = encoder_right.position() - self.prev_right;
        self.prev_left = encoder_left.position();
        self.prev_right = encoder_right.position();

        let fj = SMatrix::<f32, 3, 3>::from_row_slice(&[
            1.,
            0.,
            -(R * sinf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr)) / 2.,
            0.,
            1.,
            (R * cosf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr)) / 2.,
            0.,
            0.,
            1.,
        ]);

        let fju = SMatrix::<f32, 3, 2>::from_row_slice(&[
            (R * cosf32(self.state[2] - (R * (dwl - dwr)) / (2. * B))) / 2.
                + (R * R * sinf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr))
                    / (4. * B),
            (R * cosf32(self.state[2] - (R * (dwl - dwr)) / (2. * B))) / 2.
                - (R * R * sinf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr))
                    / (4. * B),
            (R * sinf32(self.state[2] - (R * (dwl - dwr)) / (2. * B))) / 2.
                - (R * R * cosf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr))
                    / (4. * B),
            (R * sinf32(self.state[2] - (R * (dwl - dwr)) / (2. * B))) / 2.
                + (R * R * cosf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr))
                    / (4. * B),
            -R / B,
            R / B,
        ]);

        self.state = SVector::from_column_slice(&[
            self.state[0]
                + (R * cosf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr)) / 2.,
            self.state[1]
                + (R * sinf32(self.state[2] - (R * (dwl - dwr)) / (2. * B)) * (dwl + dwr)) / 2.,
            self.state[2] - (R * (dwl - dwr)) / B,
        ]);

        self.covar = fj * self.covar * fj.transpose()
            + fju
                * SMatrix::from_diagonal(&Vector2::new(ENCODER_COVAR, ENCODER_COVAR))
                * fju.transpose();
    }

    pub fn lidar_update(&mut self, distance: f32, pose: Isometry2<f32>) {
        let hit = self.pose() * pose * Translation2::new(distance, 0.0);
        // TODO: disqualify hits in certain zones (edge and obstacles)
        let wall_x = roundf32(hit.translation.x / CELL_SIZE) * CELL_SIZE;
        let wall_y = roundf32(hit.translation.y / CELL_SIZE) * CELL_SIZE;
        let hit_angle = hit.rotation.angle().abs();
        // print!("{:?}\r\n", self.pose());
        // print!("{:?}\r\n\r\n", pose);
        print!("{}\r\n{}\r\n{}\r\n\r\n", hit_angle, (hit.translation.x - wall_x).abs(), (hit.translation.y - wall_y).abs());

        let (wn, wd) = if (hit.translation.y - wall_y).abs() < HIT_WINDOW_W
            && (hit.translation.x - wall_x).abs() > HIT_WINDOW_L
            && (-MAX_HIT_ANGLE..MAX_HIT_ANGLE).contains(&(hit_angle - f32::consts::FRAC_PI_2))
        {
            // Wall on x-axis (any x, tight y)
            (Vector2::new(0.0, 1.0), wall_y)
        } else if (hit.translation.x - wall_x).abs() < HIT_WINDOW_W
            && (hit.translation.y - wall_y).abs() > HIT_WINDOW_L
            && !(MAX_HIT_ANGLE..f32::consts::PI - MAX_HIT_ANGLE).contains(&hit_angle)
        {
            // Wall on y-axis (any y, tight x)
            (Vector2::new(1.0, 0.0), wall_x)
        } else {
            return;
        };

        let h = Self::h(self.state, pose, wn, wd);
        let hj = Self::hj(self.state, pose, wn, wd);

        let yk = distance - h;
        let sk = (hj * self.covar * hj.transpose())[0] + LIDAR_COVAR;
        let kk = self.covar * hj.transpose() / sk;

        let xk = self.state + kk * yk;
        let pk = (SMatrix::identity() - kk * hj) * self.covar;

        self.state = xk;
        self.covar = pk;
    }

    fn h(beta: SVector<f32, 3>, lp: Isometry2<f32>, wn: Vector2<f32>, wd: f32) -> f32 {
        f32::abs(
            wn[0]
                * (beta[0] + lp.translation.x * cosf32(beta[2])
                    - lp.translation.y * sinf32(beta[2]))
                - wd
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
        beta: SVector<f32, 3>,
        lp: Isometry2<f32>,
        wn: Vector2<f32>,
        wd: f32,
    ) -> SMatrix<f32, 1, 3> {
        SMatrix::from_column_slice(&[
            (wn[0]
                * f32::signum(
                    wn[0]
                        * (beta[0] + lp.translation.x * cosf32(beta[2])
                            - lp.translation.y * sinf32(beta[2]))
                        - wd
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
                    wn[0]
                        * (beta[0] + lp.translation.x * cosf32(beta[2])
                            - lp.translation.y * sinf32(beta[2]))
                        - wd
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
                wn[0]
                    * (beta[0] + lp.translation.x * cosf32(beta[2])
                        - lp.translation.y * sinf32(beta[2]))
                    - wd
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
                    wn[0]
                        * (beta[0] + lp.translation.x * cosf32(beta[2])
                            - lp.translation.y * sinf32(beta[2]))
                        - wd
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
                        wn[0] * cosf32(lp.rotation.angle() + beta[2])
                            + wn[1] * sinf32(lp.rotation.angle() + beta[2]),
                        2.0,
                    ),
        ])
    }

    pub fn pose(&self) -> Isometry2<f32> {
        Isometry2::new(Vector2::new(self.state[0], self.state[1]), self.state[2])
    }
}
