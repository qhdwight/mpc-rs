use std::iter::repeat_with;
use std::marker::PhantomData;

use na::{DMatrix, DVector, Matrix, Matrix3, RawStorage, Scalar};
use osqp::Problem;
use splines::{Interpolation, Key, Spline};

use crate::math::{diagonal_block_matrix, into_dynamic, tile};
use crate::robot::{InputVec, LinearSystem, StateVec, SystemMat};

// Q
// Constraint on the input vector
pub type ConstraintMat = Matrix3<f32>;

pub trait TimedPath {
    fn new(points: Vec<Waypoint>, dt: f32) -> Self;

    fn horizon_states(&self, t: f32, N: usize) -> DVector<StateVec>;
}

pub trait TimedPathController<Path: TimedPath, System: LinearSystem> {
    fn new(horizon: f32, dt: f32, max_velocity: InputVec, Q: ConstraintMat) -> Self;

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

pub struct MpcController<Path: TimedPath, System: LinearSystem> {
    dt: f32,
    horizon: f32,
    horizon_count: usize,
    Q: DMatrix<f32>,
    G: DMatrix<f32>,
    h: DMatrix<f32>,
    previous_u: InputVec,
    phantoms: PhantomData<(Path, System)>,
}

impl<Path: TimedPath, System: LinearSystem>
TimedPathController<Path, System> for MpcController<Path, System> {
    fn new(horizon: f32, dt: f32, max_velocity: InputVec, Q: ConstraintMat) -> Self {
        let horizon_count = f32::floor(horizon / dt) as usize;

        // Build a big block diagonal matrix that encompasses the entire horizon
        // Each "block" in the diagonal that is copied represents one state in the horizon
        // The weights are exponential as we get further out so that we converge
        let mut Q = into_dynamic(Q);
        let Q_horizon = diagonal_block_matrix(DVector::from_iterator(
            horizon_count,
            repeat_with(|| {
                let Q_temp = Q.clone();
                Q *= 2.0;
                Q_temp
            }).take(horizon_count)));
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

    fn control(&self, path: Path, x: StateVec, t: f32) -> InputVec {
        let target_states = path.horizon_states(t, self.horizon_count);
        let system = System::new(x, self.previous_u, self.dt);
        let (A, B) = system.get_system();
        let predicted_state_A_component = DVector::from_iterator(
            self.horizon_count,
            (0..self.horizon_count).scan(SystemMat::identity(), |acc, _| {
                *acc *= A;
                Some(*acc * x)
            }));

        let predicted_state_B_component = ();
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

    fn horizon_states(&self, t: f32, N: usize) -> DVector<StateVec> {
        DVector::from_iterator(N, (0..N)
            .map(|i| i as f32)
            .map(|i| self.spline.sample(t + i * self.dt).expect("Failed to interpolate")))
    }
}