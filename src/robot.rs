use na::{Matrix2x1, Matrix3, Matrix3x1, Matrix3x2};

// x: [X, Y, θ]ᵀ
pub type StateVec = Matrix3x1<f32>;
// u: [v, θ]ᵀ
pub type InputVec = Matrix2x1<f32>;
// A
pub type SystemMat = Matrix3<f32>;
// B
pub type InputMat = Matrix3x2<f32>;

pub trait System {
    fn tick(&self, u: InputVec, dt: f32) -> StateVec;

    fn new(x: StateVec, u: InputVec, dt: f32) -> Self;
}

pub trait LinearSystem: System {
    fn get_system(&self) -> (SystemMat, InputMat);
}

pub struct LinearUnicycleSystem {
    x: StateVec,
    A: SystemMat,
    B: InputMat,
}

pub struct NonlinearUnicycleSystem {
    pub x: StateVec,
}

impl System for NonlinearUnicycleSystem {
    fn tick(&self, u: InputVec, dt: f32) -> StateVec {
        let mut x = self.x.clone();
        x[0] += u[0] * f32::cos(self.x[2]) * dt; // X
        x[1] += u[0] * f32::sin(self.x[2]) * dt; // Y
        x[2] += u[1] * dt;                              // θ
        x
    }

    fn new(x: StateVec, u: InputVec, dt: f32) -> Self {
        NonlinearUnicycleSystem { x }
    }
}

impl System for LinearUnicycleSystem {
    fn tick(&self, u: InputVec, dt: f32) -> StateVec {
        self.A * self.x + self.B * u
    }

    fn new(x: StateVec, u: InputVec, dt: f32) -> Self {
        LinearUnicycleSystem {
            x,
            A: SystemMat::new(
                1.0, 0.0, -u[0] * f32::sin(x[2]) * dt,
                0.0, 1.0, u[0] * f32::cos(x[2]) * dt,
                0.0, 0.0, 1.0,
            ),
            B: InputMat::new(
                f32::cos(x[2]) * dt, 0.0,
                f32::sin(x[2]) * dt, 0.0,
                0.0, dt,
            ),
        }
    }
}

impl LinearSystem for LinearUnicycleSystem {
    fn get_system(&self) -> (SystemMat, InputMat) {
        (self.A, self.B)
    }
}

