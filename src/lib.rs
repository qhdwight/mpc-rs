extern crate nalgebra as na;

pub mod controller;

type StateVec = na::Matrix3x1<f32>;
type InputVec = na::Matrix3x1<f32>;
type SystemMat = na::Matrix3<f32>;
type InputMat = na::Matrix3x2<f32>;

trait Robot {
    fn update_state(&mut self, u: InputVec, dt: f32);

    fn from_dt(x: StateVec, u: InputVec, dt: f32) -> Self;
}

#[derive(Default)]
struct LinearUnicycleKinematics {
    // [X, Y, θ]^T State
    x: StateVec,
    A: SystemMat,
    B: InputMat,
}

impl Robot for LinearUnicycleKinematics {
    fn update_state(&mut self, u: InputVec, dt: f32) {
        self.x[0] += u[0] * f32::cos(self.x[2]) * dt; // X
        self.x[1] += u[0] * f32::sin(self.x[2]) * dt; // Y
        self.x[2] += u[1] * dt;                             // θ
    }

    fn from_dt(x: StateVec, u: InputVec, dt: f32) -> Self {
        LinearUnicycleKinematics {
            A: SystemMat::new(
                1.0, 0.0, -u[0] * f32::sin(x[2]) * dt,
                0.0, 1.0, u[0] * f32::cos(x[2]) * dt,
                1.0, 0.0, 1.0,
            ),
            B: InputMat::new(
                f32::cos(x[2]) * dt, 0.0,
                f32::sin(x[2]) * dt, 0.0,
                0.0, dt,
            ),
            ..Default::default()
        }
    }
}

