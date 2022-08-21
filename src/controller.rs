use splines::{Interpolation, Key, Spline};

use crate::robot::{InputVec, StateVec};

pub trait TimedTrajectory {
    fn lookahead_points(&self, t: f32, N: u32) -> Vec<StateVec>;

    fn new(points: Vec<Waypoint>, dt: f32) -> Self;
}

pub trait TrajectoryController {
    fn control(&self, x: StateVec, t: f32) -> InputVec;
}

pub struct Waypoint {
    pose: StateVec,
    time: f32,
}

pub struct LinearTimedTrajectory {
    spline: Spline<f32, StateVec>,
    dt: f32,
}

pub struct MpcController {}

impl TrajectoryController for MpcController {
    fn control(&self, x: StateVec, t: f32) -> InputVec {
        todo!()
    }
}

impl TimedTrajectory for LinearTimedTrajectory {
    fn lookahead_points(&self, t: f32, N: u32) -> Vec<StateVec> {
        (0..N)
            .map(|i| self.spline.sample(t + i * dt).expect("Failed to interpolate"))
            .collect()
    }

    fn new(waypoints: Vec<Waypoint>, dt: f32) -> Self {
        LinearTimedTrajectory {
            dt,
            spline: waypoints.iter()
                .map(|w| Key::new(w.time, w.pose, Interpolation::Linear))
                .collect(),
        }
    }
}