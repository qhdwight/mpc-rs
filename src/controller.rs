use std::iter::repeat_with;
use std::marker::PhantomData;

use nalgebra::{DMatrix, Matrix3};
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
    fn new(horizon: f64, dt: f64, min_input: InputVec, max_input: InputVec, Q: ConstraintMat) -> Self;

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
    horizon_count: usize,
    Q: DMatrix<f64>,
    min_input_over_horizon: DMatrix<f64>,
    max_input_over_horizon: DMatrix<f64>,
    input_constraints: DMatrix<f64>,
    previous_input: InputVec,
    phantoms: PhantomData<(Path, System)>,
}

impl<Path: TimedPath, System: LinearSystem>
TimedPathController<Path, System> for MpcController<Path, System> {
    fn new(horizon: f64, dt: f64, min_input: InputVec, max_input: InputVec, Q: ConstraintMat) -> Self {
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
        // Create the input constraint matrices
        let n_input_rows = horizon_count * max_input.nrows();
        let max_input_over_horizon = tile(&max_input, horizon_count, 1);
        let min_input_over_horizon = tile(&min_input, horizon_count, 1);
        let input_constraints = DMatrix::identity(n_input_rows, n_input_rows);
        MpcController {
            dt,
            horizon_count,
            Q: Q_horizon,
            min_input_over_horizon,
            max_input_over_horizon,
            input_constraints,
            previous_input: InputVec::zeros(),
            phantoms: PhantomData,
        }
    }

    fn control(&mut self, path: &Path, x: StateVec, t: f64) -> InputVec {
        // TODO: quite a bit of heap allocation in this function... use bump allocator?

        let goal_states = horizontal_stack(&path.horizon_states(t, self.horizon_count));

        let system = System::new(x, self.previous_input, self.dt);
        let (A, B) = system.matrices();

        let predicted_states = horizontal_stack(
            &(0..self.horizon_count).scan(SystemMat::identity(), |A_exp, _| {
                *A_exp *= &A;
                Some(*A_exp * &x)
            }).collect::<Vec<_>>()
        );

        // Compute final block row - block rows above are subsets
        let mut R_final_row_vec = (0..self.horizon_count).scan(SystemMat::identity(), |A_exp, _| {
            let next = *A_exp * &B;
            *A_exp *= &A;
            Some(next)
        }).collect::<Vec<_>>();
        R_final_row_vec.reverse();
        let R_final_row = horizontal_stack(&R_final_row_vec);
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

        let P = &R.transpose() * &self.Q * &R;
        let goal_diff = &predicted_states - &goal_states;
        let goal_diff_flattened = vertical_stack(&goal_diff.column_iter().collect::<Vec<_>>()).transpose();
        let q = &goal_diff_flattened * (&self.Q * &R);

        let P_upper_tri = P.upper_triangle();

        // Find chain of inputs which minimize error w.r.t. path over the entire horizon
        let mut minimize_horizon_error_problem = Problem::new(
            into_sparse(&P_upper_tri),
            q.as_slice(),
            into_sparse(&self.input_constraints),
            self.min_input_over_horizon.as_slice(),
            self.max_input_over_horizon.as_slice(),
            &Settings::default().verbose(false),
        ).expect("Failed to setup problem");

        let minimize_horizon_error_solution = minimize_horizon_error_problem.solve();

        let next_input = match minimize_horizon_error_solution.x() {
            Some(optimal_input_chain_over_horizon) => {
                // Only take the most recent input from the chain
                InputVec::from_column_slice(&optimal_input_chain_over_horizon[..2])
            }
            None => InputVec::zeros()
        };

        self.previous_input = next_input;

        next_input
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
            .map(|i| self.spline.clamped_sample(t + i as f64 * self.dt).expect("Failed to sample"))
            .collect()
    }
}
