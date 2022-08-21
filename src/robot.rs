// x: [X, Y, θ]ᵀ
pub type StateVec = na::Matrix3x1<f32>;
// u: [v, θ]ᵀ
pub type InputVec = na::Matrix2x1<f32>;
// A
pub type SystemMat = na::Matrix3<f32>;
// B
pub type InputMat = na::Matrix3x2<f32>;

trait Robot {
    fn tick(&self, u: InputVec, T: f32) -> StateVec;

    fn new(x: StateVec, u: InputVec, T: f32) -> Self;
}

pub struct LinearUnicycleKinematics {
    x: StateVec,
    A: SystemMat,
    B: InputMat,
}

pub struct UnicycleKinematics {
    pub x: StateVec,
}

impl Robot for UnicycleKinematics {
    fn tick(&self, u: InputVec, T: f32) -> StateVec {
        let mut x = self.x.clone();
        x[0] += u[0] * f32::cos(self.x[2]) * T; // X
        x[1] += u[0] * f32::sin(self.x[2]) * T; // Y
        x[2] += u[1] * T;                              // θ
        x
    }

    fn new(x: StateVec, u: InputVec, T: f32) -> Self {
        return UnicycleKinematics { x };
    }
}

impl Robot for LinearUnicycleKinematics {
    fn tick(&self, u: InputVec, T: f32) -> StateVec {
        self.A * self.x + self.B * u
    }

    fn new(x: StateVec, u: InputVec, T: f32) -> Self {
        LinearUnicycleKinematics {
            x,
            A: SystemMat::new(
                1.0, 0.0, -u[0] * f32::sin(x[2]) * T,
                0.0, 1.0, u[0] * f32::cos(x[2]) * T,
                0.0, 0.0, 1.0,
            ),
            B: InputMat::new(
                f32::cos(x[2]) * T, 0.0,
                f32::sin(x[2]) * T, 0.0,
                0.0, T,
            ),
        }
    }
}

