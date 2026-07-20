use na::{Isometry, Isometry2, SMatrix, SVector, zero};

use crate::{DT, concat};

struct MotionManager {
    state_current: [SVector<f32, 2>; 2],
    state_target: [SVector<f32, 2>; 2],
    remaining_time: f32,
}

impl MotionManager {
    pub fn new() -> Self {
        Self {
            state_current: [zero(); 2],
            state_target: [zero(); 2],
            remaining_time: 0.0,
        }
    }

    pub fn update(&mut self, current_pose: Isometry2<f32>) -> (f32, f32) {
        if self.remaining_time <= DT {
            self.state_current = self.state_target;
            self.remaining_time = 0.;
        } else {
            self.state_current = [
                Self::spline_step(self.state_current[0], self.state_target[0], self.remaining_time),
                Self::spline_step(self.state_current[1], self.state_target[1], self.remaining_time),
            ];
            self.remaining_time -= DT;
        }

        (0.0, 0.0)
    }

    pub fn set_target(&mut self, pose: Isometry2<f32>, velocity: f32) {
    }

    fn spline_step(si: SVector<f32, 2>, sf: SVector<f32, 2>, pd: f32) -> SVector<f32, 2> {
        let pd_sq = pd * pd;
        let pd_cb = pd_sq * pd;
        let dt_sq = DT * DT;
        let dt_cb = DT * DT;

        SMatrix::<f32, 2, 4>::from_row_slice(&[
            2. * pd_cb - 3. * pd_sq * DT + dt_cb,
            pd_sq - 2. * pd * DT + dt_sq,
            1.,
            DT - pd,
            3. * dt_sq - 3. * pd_sq,
            2. * DT - 2. * pd,
            0.,
            1.,
        ]) * SVector::from_iterator(concat(&si.data.0[0], &sf.data.0[0]))
    }
}
