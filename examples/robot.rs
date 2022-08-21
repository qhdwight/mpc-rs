use bevy::prelude::*;

use mpc_rs::controller::MpcController;
use mpc_rs::robot::*;

#[derive(Component)]
struct Robot {
    kinematics: UnicycleKinematics,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(MpcController {})
        .insert_resource(WindowDescriptor {
            title: "Robot Simulator".to_string(),
            ..default()
        })
        .add_startup_system(setup)
        .add_system(tick)
        .add_plugins(DefaultPlugins)
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
    });
    commands.spawn().insert(Robot {
        kinematics: UnicycleKinematics { x: StateVec::new(2.0, 2.0, 0.0) }
    });
}

fn tick(robot: Query<&Robot>, controller: ResMut<MpcController>) {}