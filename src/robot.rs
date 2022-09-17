use nalgebra::{Matrix2x1, Matrix3, Matrix3x1, Matrix3x2};

// x: [X, Y, θ]ᵀ
pub type StateVec = Matrix3x1<f64>;
// u: [v, θ]ᵀ
pub type InputVec = Matrix2x1<f64>;
// A
pub type SystemMat = Matrix3<f64>;
// B
pub type InputMat = Matrix3x2<f64>;

pub trait System {
    fn tick(&self, u: InputVec, dt: f64) -> StateVec;

    fn new(x: StateVec, u: InputVec, dt: f64) -> Self;
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
    fn tick(&self, u: InputVec, dt: f64) -> StateVec {
        let mut x = self.x.clone();
        x[0] += u[0] * f64::cos(self.x[2]) * dt; // X
        x[1] += u[0] * f64::sin(self.x[2]) * dt; // Y
        x[2] += u[1] * dt;                            // θ
        x
    }

    fn new(x: StateVec, _u: InputVec, _dt: f64) -> Self {
        NonlinearUnicycleSystem { x }
    }
}

impl System for LinearUnicycleSystem {
    fn tick(&self, u: InputVec, _dt: f64) -> StateVec {
        self.A * self.x + self.B * u
    }

    fn new(x: StateVec, u: InputVec, dt: f64) -> Self {
        LinearUnicycleSystem {
            x,
            A: SystemMat::new(
                1.0, 0.0, -u[0] * f64::sin(x[2]) * dt,
                0.0, 1.0, u[0] * f64::cos(x[2]) * dt,
                0.0, 0.0, 1.0,
            ),
            B: InputMat::new(
                f64::cos(x[2]) * dt, 0.0,
                f64::sin(x[2]) * dt, 0.0,
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
