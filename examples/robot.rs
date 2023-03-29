use std::f64::consts::TAU;

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
};
use bevy_prototype_lyon::prelude::*;

use mpc_rs::{
    controller::{ConstraintMat, LinearTimedPath, MpcController, TimedPath, TimedPathController, Waypoint},
    robot::{InputVec, LinearUnicycleSystem, NonlinearUnicycleSystem, StateVec, System},
};

#[derive(Component)]
struct Robot(NonlinearUnicycleSystem, Vec<Vec2>);

#[derive(Component)]
struct Controller(MpcController<LinearTimedPath, LinearUnicycleSystem>);

#[derive(Component)]
struct Trajectory(LinearTimedPath);

#[derive(Default, Resource)]
struct Simulation {
    elapsed_seconds: f64,
}

const TICK_DELTA_SECONDS: f64 = 0.05;
const HORIZON_SECONDS: f64 = 0.2;
const WORLD_TO_SCREEN: f32 = 20.0;
const SPRITE_SIZE: f32 = 32.0;


fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .add_startup_system(setup)
        .add_system(tick)
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Robot Simulator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugin(ShapePlugin)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(Simulation::default())
        .run();
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
) {
    let waypoints = vec![
        Waypoint { pose: StateVec::new(0.0, 0.0, 0.0), time: 0.0 },
        Waypoint { pose: StateVec::new(10.0, 0.0, 0.0), time: 10.0 },
        Waypoint { pose: StateVec::new(10.0, 1.0, TAU / 4.0), time: 11.0 },
        Waypoint { pose: StateVec::new(10.0, 10.0, TAU / 4.0), time: 20.0 },
        Waypoint { pose: StateVec::new(9.0, 10.0, TAU / 2.0), time: 21.0 },
        Waypoint { pose: StateVec::new(-10.0, 10.0, TAU / 2.0), time: 40.0 },
        Waypoint { pose: StateVec::new(-10.0, 9.0, 3.0 * TAU / 4.0), time: 41.0 },
        Waypoint { pose: StateVec::new(-10.0, -10.0, 3.0 * TAU / 4.0), time: 60.0 },
    ];
    let mut trajectory_path_builder = PathBuilder::new();
    trajectory_path_builder.move_to(Vec2::ZERO);
    for waypoint in &waypoints {
        trajectory_path_builder.line_to(WORLD_TO_SCREEN * Vec2::new(waypoint.pose.x as f32, waypoint.pose.y as f32));
    }
    commands.spawn((
        ShapeBundle {
            path: GeometryBuilder::build_as(&trajectory_path_builder.build()),
            ..default()
        },
        Stroke::color(Color::GREEN),
    ));
    commands.spawn(Camera2dBundle::default());
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                custom_size: Some(Vec2::new(SPRITE_SIZE, SPRITE_SIZE)),
                ..default()
            },
            texture: asset_server.load("robot.png"),
            ..default()
        },
        Robot(NonlinearUnicycleSystem {
            x: StateVec::new(0.0, 0.0, 0.0)
        }, Vec::new()),
        Controller(MpcController::new(
            HORIZON_SECONDS, TICK_DELTA_SECONDS,
            InputVec::new(-1.0, -0.3),
            InputVec::new(1.0, 0.3),
            ConstraintMat::new(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 50.0,
            ),
        )),
        Trajectory(LinearTimedPath::new(
            waypoints, TICK_DELTA_SECONDS,
        ))
    ));
}

fn tick(mut time: ResMut<Simulation>, mut query: Query<(&mut Robot, &mut Transform, &mut Controller, &Trajectory)>) {
    for (mut robot, mut transform, mut controller, trajectory) in &mut query {
        let u = controller.0.control(&trajectory.0, robot.0.x, time.elapsed_seconds);
        // let u = InputVec::zeros();
        robot.0.x = robot.0.tick(u, TICK_DELTA_SECONDS);
        time.elapsed_seconds += TICK_DELTA_SECONDS;
        let pose = &robot.0.x;
        transform.translation = WORLD_TO_SCREEN * Vec3::new(pose.x as f32, pose.y as f32, 0.0);
        transform.rotation = Quat::from_axis_angle(Vec3::Z, (pose.z - TAU / 4.0) as f32);
    }
}
