use std::iter::repeat_with;
use std::marker::PhantomData;

use nalgebra::{DMatrix, DVector, Matrix3};
use osqp::{Problem, Settings};
use splines::{Interpolation, Key, Spline};

use crate::math::{diagonal_block_matrix, horizontal_stack, into_dynamic, into_sparse, tile, vertical_stack};
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

    fn control(&mut self, path: &Path, x: StateVec, t: f64) -> InputVec;
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
    previous_control: InputVec,
    phantoms: PhantomData<(Path, System)>,
}

impl<Path: TimedPath, System: LinearSystem>
TimedPathController<Path, System> for MpcController<Path, System> {
    fn new(horizon: f64, dt: f64, max_velocity: InputVec, Q: ConstraintMat) -> Self {
        let horizon_count = f64::floor(horizon / dt) as usize;

        // Build a big block diagonal matrix that encompasses the entire horizon
        // Each "block" in the diagonal that is copied represents one state in the horizon
        // The weights are exponential as we get further out so that we converge
        let mut Q = into_dynamic(&Q);
        let Q_horizon = diagonal_block_matrix(
            &repeat_with(|| {
                let Q_temp = Q.clone();
                Q *= 2.0;
                Q_temp
            }).take(horizon_count).collect::<Vec<_>>()
        );
        // Create the control constraint matrices
        let M = horizon_count * 2;
        let I = DMatrix::identity(M, M);
        let nI = -&I;
        let G = vertical_stack(&[I, nI]);
        let h = tile(&max_velocity, horizon_count * 2, 1);
        MpcController {
            dt,
            horizon,
            horizon_count,
            Q: Q_horizon,
            G,
            h,
            previous_control: InputVec::zeros(),
            phantoms: PhantomData,
        }
    }

    fn control(&mut self, path: &Path, x: StateVec, t: f64) -> InputVec {
        let target_states = horizontal_stack(&path.horizon_states(t, self.horizon_count));

        let system = System::new(x, self.previous_control, self.dt);
        let (A, B) = system.get_system();

        let alpha = horizontal_stack(
            &(0..self.horizon_count).scan(SystemMat::identity(), |A_exp, _| {
                *A_exp *= &A;
                Some(*A_exp * &x)
            }).collect::<Vec<_>>()
        );

        // Compute final block row - block rows above are subsets
        let R_final_row = horizontal_stack(
            &(0..self.horizon_count).scan(SystemMat::identity(), |A_exp, _| {
                let next = *A_exp * &B;
                *A_exp *= &A;
                Some(next)
            }).collect::<Vec<_>>()
        );
        // Create block triangular matrix
        let R = DMatrix::from_fn(self.horizon_count * B.nrows(), R_final_row.ncols(), |r, c| {
            let ir = r / B.nrows();
            let ic = c / B.ncols();
            if (self.horizon_count - ic) + ir < self.horizon_count {
                0.0
            } else {
                R_final_row[(r % B.nrows(), c % B.ncols())]
            }
        });

        // let R = tile(R_final_row, self.horizon_count, 1).upper_triangle();

        let P = &R.transpose() * &self.Q * &R;
        let goal_diff = &alpha - &target_states;
        let goal_diff_flattened = vertical_stack(&goal_diff.column_iter().collect::<Vec<_>>()).transpose();
        let q = &goal_diff_flattened * (&self.Q * &R);

        let P_upper_tri = P.upper_triangle();

        let control_min_constraint = DVector::zeros(self.h.nrows());
        let control_max_constraint = &self.h;

        // Find chain of control inputs which minimize error w.r.t. path over the entire horizon
        let mut minimize_horizon_error_problem = Problem::new(
            into_sparse(&P_upper_tri),
            q.as_slice(),
            into_sparse(&self.G),
            control_min_constraint.as_slice(),
            control_max_constraint.as_slice(),
            &Settings::default().verbose(false),
        ).expect("Failed to setup problem");

        let minimize_horizon_error_solution = minimize_horizon_error_problem.solve();

        let next_control = match minimize_horizon_error_solution.x() {
            Some(optimal_control_over_horizon) => {
                // Only take the most recent control from the chain
                InputVec::from_column_slice(&optimal_control_over_horizon[..2])
            }
            None => InputVec::zeros()
        };

        self.previous_control = next_control;

        next_control
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
            .map(|i| self.spline.sample(t + i as f64 * self.dt).expect("Failed to interpolate"))
            .collect()
    }
}
