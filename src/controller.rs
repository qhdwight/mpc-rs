use std::iter::repeat_with;
use std::marker::PhantomData;

use nalgebra::{DMatrix, Matrix3};
use osqp::{CscMatrix, Settings};
use splines::{Interpolation, Key, Spline};

use crate::math::{diagonal_block_matrix, horizontal_stack, into_dynamic, tile};
use crate::robot::{InputVec, LinearSystem, StateVec, SystemMat};

// Q
// Constraint on the input vector
pub type ConstraintMat = Matrix3<f64>;

pub trait TimedPath {
    fn new(points: Vec<Waypoint>, dt: f64) -> Self;

    fn horizon_states(&self, t: f64, N: usize) -> Vec<StateVec>;
}

pub trait TimedPathController<Path: TimedPath, System: LinearSystem> {
    fn new(horizon: f64, dt: f64, max_velocity: InputVec, Q: ConstraintMat) -> Self;

    fn control(&self, path: &Path, x: StateVec, t: f64) -> InputVec;
}

pub struct Waypoint {
    pub pose: StateVec,
    pub time: f64,
}

pub struct LinearTimedPath {
    dt: f64,
    spline: Spline<f64, StateVec>,
}

pub struct MpcController<Path: TimedPath, System: LinearSystem> {
    dt: f64,
    horizon: f64,
    horizon_count: usize,
    Q: DMatrix<f64>,
    G: DMatrix<f64>,
    h: DMatrix<f64>,
    previous_u: InputVec,
    phantoms: PhantomData<(Path, System)>,
}

impl<Path: TimedPath, System: LinearSystem>
TimedPathController<Path, System> for MpcController<Path, System> {
    fn new(horizon: f64, dt: f64, max_velocity: InputVec, Q: ConstraintMat) -> Self {
        let horizon_count = f64::floor(horizon / dt) as usize;

        // Build a big block diagonal matrix that encompasses the entire horizon
        // Each "block" in the diagonal that is copied represents one state in the horizon
        // The weights are exponential as we get further out so that we converge
        let mut Q = into_dynamic(Q);
        let Q_horizon = diagonal_block_matrix(
            repeat_with(|| {
                let Q_temp = Q.clone();
                Q *= 2.0;
                Q_temp
            }).take(horizon_count).collect()
        );
        // Create the velocity constraint matrices
        let M = horizon_count * 2;
        let mut G = DMatrix::from_element(M * 2, M, 0.0);
        let I = DMatrix::identity(M, M);
        G.index_mut((..M, ..M)).copy_from(&I);
        G.index_mut((M..M * 2, ..M)).copy_from(&-I);
        let h = tile(max_velocity, horizon_count * 2, 1);
        MpcController {
            dt,
            horizon,
            horizon_count,
            Q: Q_horizon,
            G,
            h,
            previous_u: InputVec::zeros(),
            phantoms: PhantomData,
        }
    }

    fn control(&self, path: &Path, x: StateVec, t: f64) -> InputVec {
        let target_states = horizontal_stack(path.horizon_states(t, self.horizon_count));

        let system = System::new(x, self.previous_u, self.dt);
        let (A, B) = system.get_system();

        let alpha = horizontal_stack(
            (0..self.horizon_count).scan(SystemMat::identity(), |A_exp, _| {
                *A_exp *= &A;
                Some(*A_exp * &x)
            }).collect()
        );

        let R_row = horizontal_stack(
            (0..self.horizon_count).scan(SystemMat::identity(), |A_exp, _| {
                let next = *A_exp * &B;
                *A_exp *= &A;
                Some(next)
            }).collect()
        );
        let R = R_row.clone_owned();

        let P = &R.transpose() * &self.Q * &R;
        let goal_diff = &alpha - &target_states;
        let goal_diff_flattened = horizontal_stack(goal_diff.row_iter().collect());
        let q = &goal_diff_flattened * (&self.Q * &R);

        println!("{}, {}, {}", R, P, q);

        let P = CscMatrix::from_column_iter_dense(P.nrows(), P.ncols(), P.iter().cloned());

        let settings = Settings::default().verbose(true);

        // let mut prob = Problem::new(P, q, self.G, self.h, &settings).expect("Failed to setup problem");

        InputVec::zeros()
    }
}

impl TimedPath for LinearTimedPath {
    fn new(waypoints: Vec<Waypoint>, dt: f64) -> Self {
        LinearTimedPath {
            dt,
            spline: Spline::from_iter(waypoints.iter()
                .map(|w| Key::new(w.time, w.pose, Interpolation::Linear))),
        }
    }

    fn horizon_states(&self, t: f64, N: usize) -> Vec<StateVec> {
        (0..N)
            .map(|i| i as f64)
            .map(|i| self.spline.sample(t + i * self.dt).expect("Failed to interpolate"))
            .collect()
    }
}