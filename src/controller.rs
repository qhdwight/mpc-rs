use std::marker::PhantomData;

use na::{DMatrix, DVector, Matrix3, Vector, Vector3};
use splines::{Interpolation, Key, Spline};

use crate::robot::{InputVec, LinearUnicycleKinematics, RobotKinematics, StateVec};

// Q
pub type ConstraintMat = Matrix3<f32>;

pub trait TimedPath {
    fn new(points: Vec<Waypoint>, dt: f32) -> Self;

    fn horizon_states(&self, t: f32, N: u32) -> Vec<StateVec>;
}

pub trait TimedPathController<Path: TimedPath> {
    fn new(horizon: f32, dt: f32, max_velocity: f32, Q: ConstraintMat) -> Self;

    fn control(&self, path: Path, x: StateVec, t: f32) -> InputVec;
}

pub struct Waypoint {
    pose: StateVec,
    time: f32,
}

pub struct LinearTimedPath {
    dt: f32,
    spline: Spline<f32, StateVec>,
}

pub struct MpcController<Path: TimedPath> {
    dt: f32,
    horizon: f32,
    N: u32,
    Q: DMatrix<f32>,
    previous_u: InputVec,
    phantom: PhantomData<Path>,
}

impl<Path: TimedPath> TimedPathController<Path> for MpcController<Path> {
    fn new(horizon: f32, dt: f32, max_velocity: f32, Q: ConstraintMat) -> Self {
        let N = f32::floor(horizon / dt) as u32;
        let M = Q.ncols() * N as usize;
        let mut Q_blocked = DMatrix::zeros(M, M);
        let mut Q_i = Q.clone();
        for i in 0..(N as usize) {
            let j = i + Q_i.ncols();
            Q_blocked.index_mut((i..j, i..j)).copy_from(&Q_i);
            Q_i *= Q;
        }
        MpcController {
            dt,
            horizon,
            N,
            Q: Q_blocked,
            previous_u: InputVec::zeros(),
            phantom: PhantomData,
        }
    }

    fn control(&self, path: Path, x: StateVec, t: f32) -> InputVec {
        let horizon_states = path.horizon_states(t, self.N);
        let kinematics = LinearUnicycleKinematics::new(x, self.previous_u, self.dt);

        todo!()
    }
}

impl TimedPath for LinearTimedPath {
    fn new(waypoints: Vec<Waypoint>, dt: f32) -> Self {
        LinearTimedPath {
            dt,
            spline: Spline::from_iter(waypoints.iter()
                .map(|w| Key::new(w.time, w.pose, Interpolation::Linear))),
        }
    }

    fn horizon_states(&self, t: f32, N: u32) -> Vec<StateVec> {
        (0..N)
            .map(|i| i as f32)
            .map(|i| self.spline.sample(t + i * self.dt).expect("Failed to interpolate"))
            .collect()
    }
}