use std::f64::consts::TAU;

use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::prelude::*;

use mpc_rs::controller::{ConstraintMat, LinearTimedPath, MpcController, TimedPath, TimedPathController, Waypoint};
use mpc_rs::robot::{InputVec, LinearUnicycleSystem, NonlinearUnicycleSystem, StateVec, System};

#[derive(Component)]
struct Robot(NonlinearUnicycleSystem);

#[derive(Component)]
struct Controller(MpcController<LinearTimedPath, LinearUnicycleSystem>);

#[derive(Component)]
struct Trajectory(LinearTimedPath);

#[derive(Default)]
struct SimTime {
    time: f64,
}

const TICK_DELTA: f64 = 0.05;
const HORIZON: f64 = 0.2;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(WindowDescriptor {
            title: "Robot Simulator".to_string(),
            ..default()
        })
        .add_startup_system(setup)
        .add_system(tick)
        .add_plugins(DefaultPlugins)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(SimTime::default())
        .run();
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn_bundle(Camera2dBundle::default());
    commands.spawn_bundle(SpriteBundle {
        sprite: Sprite {
            custom_size: Some(Vec2::new(16.0, 16.0)),
            ..default()
        },
        texture: asset_server.load("robot.png"),
        transform: Transform::from_xyz(20.0, 20.0, 0.0),
        ..default()
    })
        .insert(Robot(NonlinearUnicycleSystem {
            x: StateVec::new(0.0, 0.0, 0.0)
        }))
        .insert(Controller(MpcController::new(
            HORIZON, TICK_DELTA, InputVec::new(1.0, 0.3), ConstraintMat::new(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 50.0,
            ),
        )))
        .insert(Trajectory(LinearTimedPath::new(
            vec![
                Waypoint { pose: StateVec::new(0.0, 0.0, 0.0), time: 0.0 },
                Waypoint { pose: StateVec::new(10.0, 0.0, 0.0), time: 10.0 },
                Waypoint { pose: StateVec::new(10.0, 1.0, TAU / 4.0), time: 11.0 },
                Waypoint { pose: StateVec::new(10.0, 10.0, TAU / 4.0), time: 20.0 },
                Waypoint { pose: StateVec::new(9.0, 10.0, TAU / 2.0), time: 21.0 },
                Waypoint { pose: StateVec::new(-10.0, 10.0, TAU / 2.0), time: 40.0 },
                Waypoint { pose: StateVec::new(-10.0, 9.0, TAU / 4.0), time: 41.0 },
                Waypoint { pose: StateVec::new(-10.0, -10.0, TAU / 4.0), time: 60.0 },
            ], TICK_DELTA,
        )));
}

fn tick(mut time: ResMut<SimTime>, mut query: Query<(&mut Robot, &mut Transform, &mut Controller, &Trajectory)>) {
    for (mut robot, mut transform, mut controller, trajectory) in &mut query {
        let mut robot: Mut<Robot> = robot;
        let mut transform: Mut<Transform> = transform;

        let u = controller.0.control(&trajectory.0, robot.0.x, time.time);
        robot.0.x = robot.0.tick(u, TICK_DELTA);
        time.time += TICK_DELTA;
        transform.translation = 500.0 * Vec3::new(robot.0.x.x as f32, robot.0.x.y as f32, 0.0);
    }
}
