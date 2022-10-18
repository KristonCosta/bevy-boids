use bevy::{prelude::*, sprite::MaterialMesh2dBundle, window::WindowResized};
use rand::prelude::*;

#[derive(Component)]
struct Boid;

#[derive(Component)]
struct Velocity(Vec3);

struct Level {
    area: Vec2,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(Level {
            area: Vec2::new(500.0, 500.0),
        })
        .add_system(update_position)
        .add_system(on_resize_system)
        .add_startup_system(setup)
        .add_startup_system(spawn_boids)
        .run();
}

fn setup(mut commands: Commands, mut level: ResMut<Level>, windows: ResMut<Windows>) {
    commands.spawn().insert_bundle(Camera2dBundle::default());
    level.area = Vec2::new(windows.primary().width(), windows.primary().height());
}

fn spawn_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    level: Res<Level>,
) {
    let mut rng = thread_rng();
    let width = level.area.x / 2.0;
    let height = level.area.y / 2.0;
    for _ in 0..100 {
        let pos_x = rng.gen_range(-width..width);
        let vel_x = rng.gen_range(-10.0..10.0) * 10.0;

        let pos_y = rng.gen_range(-height..height);
        let vel_y = rng.gen_range(-10.0..10.0) * 10.0;

        commands
            .spawn()
            .insert_bundle(MaterialMesh2dBundle {
                mesh: meshes.add(shape::RegularPolygon::new(20., 3).into()).into(),
                material: materials.add(ColorMaterial::from(Color::PURPLE)),
                transform: Transform::from_translation(Vec3::new(pos_x, pos_y, 0.)),
                ..default()
            })
            .insert(Velocity(Vec3::new(vel_x, vel_y, 0.0)));
    }
}

fn update_position(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &Velocity)>,
    level: Res<Level>,
) {
    for (mut pos, vel) in query.iter_mut() {
        pos.translation += vel.0 * time.delta_seconds();
        let width = level.area.x / 2.0;
        let height = level.area.y / 2.0;
        if pos.translation.x < -width {
            pos.translation.x += level.area.x;
        }
        if pos.translation.y < -height {
            pos.translation.y += level.area.y;
        }
        if pos.translation.x > width {
            pos.translation.x -= level.area.x
        }
        if pos.translation.y > height {
            pos.translation.y -= level.area.y
        }
    }
}

fn on_resize_system(mut level: ResMut<Level>, mut resize_reader: EventReader<WindowResized>) {
    for e in resize_reader.iter() {
        level.area = Vec2::new(e.width, e.height)
    }
}
