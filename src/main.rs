use bevy::{math::Vec3Swizzles, prelude::*, sprite::MaterialMesh2dBundle, window::WindowResized};
use rand::prelude::*;

#[derive(Component)]
struct Boid;

#[derive(Component)]
struct Velocity(Vec3);

enum SteeringForce {
    Separation(Vec3),
}

#[derive(Component)]
struct SteeringForces(Vec<SteeringForce>);

impl Default for SteeringForces {
    fn default() -> Self {
        Self(Vec::with_capacity(10))
    }
}

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
        .add_system(apply_separation)
        .add_system(apply_steering_force.after(apply_separation))
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
    for i in 0..20 {
        let pos_x = rng.gen_range(-width..width);
        let vel_x = rng.gen_range(-10.0..10.0) * 10.0;

        let pos_y = rng.gen_range(-height..height);
        let vel_y = rng.gen_range(-10.0..10.0) * 10.0;

        let color = if i == 0 {
            ColorMaterial::from(Color::PURPLE)
        } else {
            ColorMaterial::from(Color::BLUE)
        };

        commands
            .spawn()
            .insert_bundle(MaterialMesh2dBundle {
                mesh: meshes.add(shape::RegularPolygon::new(20., 3).into()).into(),
                material: materials.add(color),
                transform: Transform::from_translation(Vec3::new(pos_x, pos_y, 0.)),
                ..default()
            })
            .insert(Boid)
            .insert(Velocity(Vec3::new(vel_x, vel_y, 0.0)))
            .insert(SteeringForces::default());
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
        let current_direction = (pos.rotation * Vec3::Y).xy();

        let desired_direction = vel.0.truncate().normalize();
        let angle = current_direction
            .dot(desired_direction)
            .clamp(-1.0, 1.0)
            .acos();

        pos.rotate_z(angle);
    }
}

fn shortest_path_to_target(level: &Level, origin: Vec3, target: Vec3) -> Vec3 {
    let width = level.area.x / 2.0;
    let height = level.area.y / 2.0;

    let mut calculated_distance = origin - target;

    if calculated_distance.x > width {
        // Go the other way around
        calculated_distance.x -= level.area.x;
    }
    if calculated_distance.x < -width {
        calculated_distance.x += level.area.x;
    }

    if calculated_distance.y > height {
        // Go the other way around
        calculated_distance.y -= level.area.y;
    }
    if calculated_distance.y < -height {
        calculated_distance.y += level.area.y;
    }

    calculated_distance
}

fn apply_steering_force(time: Res<Time>, mut query: Query<(&mut Velocity, &mut SteeringForces)>) {
    for (mut vel, mut forces) in query.iter_mut() {
        for force in forces.0.drain(..) {
            match force {
                SteeringForce::Separation(force) => {
                    vel.0 += force.clone() * time.delta_seconds();
                    vel.0 = vel.0.clamp_length(-100.0, 100.0);
                }
            }
        }
    }
}

fn apply_separation(
    level: Res<Level>,
    position_query: Query<(Entity, &Transform), With<Boid>>,
    mut steering_force_query: Query<&mut SteeringForces>,
) {
    for (target, target_position) in position_query.iter() {
        let mut steering_force = Vec3::ZERO;
        for (neighbour, neighbour_position) in position_query.iter() {
            if target == neighbour {
                continue;
            }
            let distance = shortest_path_to_target(
                &level,
                target_position.translation,
                neighbour_position.translation,
            );
            if distance.length() == 0.0 {
                continue;
            }
            steering_force += distance.normalize() / distance.length();
        }
        if steering_force.length() > 0.0 {
            let mut current_forces = steering_force_query.get_mut(target).unwrap();
            current_forces
                .0
                .push(SteeringForce::Separation(steering_force))
        }
    }
}

fn on_resize_system(mut level: ResMut<Level>, mut resize_reader: EventReader<WindowResized>) {
    for e in resize_reader.iter() {
        level.area = Vec2::new(e.width, e.height)
    }
}
