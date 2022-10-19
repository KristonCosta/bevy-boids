use bevy::{math::Vec3Swizzles, prelude::*, sprite::MaterialMesh2dBundle, window::WindowResized};
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use rand::prelude::*;

#[derive(Component)]
struct Boid;

#[derive(Component)]
struct Velocity(Vec3);

#[derive(Inspectable)]
struct SteeringForceConf {
    alignment: f32,
    separation: f32,
    cohesion: f32,
    seek: f32,
    #[inspectable(min = 10.0, max = 1000.0)]
    max_speed: f32,
    neighbourhood_size: f32,
    steering_force_tweaker: f32,
    max_steering_force: f32,
}

impl Default for SteeringForceConf {
    fn default() -> Self {
        Self {
            alignment: 1.0,
            separation: 2.0,
            cohesion: 2.0,
            seek: 1.0,
            max_speed: 200.0,
            neighbourhood_size: 500.0,
            steering_force_tweaker: 200.0,
            max_steering_force: 2.0,
        }
    }
}

impl Velocity {
    pub fn heading(&self) -> Vec3 {
        self.0.normalize()
    }
}

enum SteeringForce {
    Separation(Vec3),
    Alignment(Vec3),
    Cohesion(Vec3),
    Seek(Vec3),
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
        .add_plugin(InspectorPlugin::<SteeringForceConf>::new())
        .insert_resource(Level {
            area: Vec2::new(500.0, 500.0),
        })
        .add_system(update_position)
        .add_system(on_resize_system)
        .add_startup_system(setup)
        .add_startup_system(spawn_boids)
        .add_system(apply_separation)
        .add_system(apply_alignment)
        .add_system(apply_cohesion)
        .add_system(apply_seek)
        .add_system(
            apply_steering_force
                .after(apply_separation)
                .after(apply_alignment)
                .after(apply_cohesion)
                .after(apply_seek),
        )
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
    for i in 0..400 {
        let pos_x = rng.gen_range(-width..width);
        let vel_x = rng.gen_range(-10.0..10.0) * 10.0;

        let pos_y = rng.gen_range(-height..height);
        let vel_y = rng.gen_range(-10.0..10.0) * 10.0 + 10.0;

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
        let dot = current_direction.dot(desired_direction);
        if (dot - 1.0).abs() < f32::EPSILON {
            continue;
        }

        let right = (pos.rotation * Vec3::X).xy();
        let right_dot = right.dot(desired_direction);
        let rotation_sign = -f32::copysign(1.0, right_dot);
        let angle = dot.clamp(-1.0, 1.0).acos();

        pos.rotate_z(rotation_sign * angle);
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

fn apply_steering_force(
    time: Res<Time>,
    mut query: Query<(&mut Velocity, &mut SteeringForces)>,
    conf: Res<SteeringForceConf>,
) {
    for (mut vel, mut forces) in query.iter_mut() {
        let mut combined = Vec3::ZERO;
        for force in forces.0.drain(..) {
            let force = match force {
                SteeringForce::Separation(force) => force * conf.separation,
                SteeringForce::Alignment(force) => force * conf.alignment,
                SteeringForce::Cohesion(force) => force * conf.cohesion,
                SteeringForce::Seek(force) => force * conf.seek,
            } * conf.steering_force_tweaker;
            combined += force;

            // vel.0 = vel.0.normalize() * conf.max_speed;
        }
        combined = combined.clamp_length_max(conf.max_steering_force * conf.steering_force_tweaker);
        vel.0 += combined * time.delta_seconds();
        vel.0 = vel.0.clamp_length(10.0, conf.max_speed);
    }
}

fn apply_alignment(
    level: Res<Level>,
    conf: Res<SteeringForceConf>,
    velocity_query: Query<(Entity, &Transform, &Velocity), With<Boid>>,
    mut steering_force_query: Query<(Entity, &mut SteeringForces)>,
) {
    for (target, mut forces) in steering_force_query.iter_mut() {
        let mut average_heading = Vec3::ZERO;
        let mut neighbours = 0;
        let (_, target_transform, target_vel) = velocity_query.get(target).unwrap();
        for (neighbour, neighbour_transform, neighbour_vel) in velocity_query.iter() {
            if shortest_path_to_target(
                &level,
                target_transform.translation,
                neighbour_transform.translation,
            )
            .length_squared()
                > conf.neighbourhood_size
                || target == neighbour
            {
                continue;
            }
            average_heading += neighbour_vel.heading();
            neighbours += 1;
        }

        if neighbours > 0 {
            let corrected_heading = average_heading - target_vel.0;
            let normalized_heading = corrected_heading / (neighbours as f32);

            forces.0.push(SteeringForce::Alignment(normalized_heading));
        }
    }
}

fn apply_separation(
    level: Res<Level>,
    conf: Res<SteeringForceConf>,
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
            if distance.length_squared() == 0.0
                || distance.length_squared() > conf.neighbourhood_size
            {
                continue;
            }
            steering_force += distance.normalize() / distance.length();
        }
        if steering_force.length_squared() > 0.0 {
            let mut current_forces = steering_force_query.get_mut(target).unwrap();
            current_forces
                .0
                .push(SteeringForce::Separation(steering_force))
        }
    }
}

fn apply_seek(
    buttons: Res<Input<MouseButton>>,
    level: Res<Level>,
    windows: Res<Windows>,
    conf: Res<SteeringForceConf>,
    query: Query<(Entity, &Transform, &Velocity), With<Boid>>,
    mut steering_force_query: Query<&mut SteeringForces>,
) {
    if buttons.pressed(MouseButton::Left) {
        let cursor = windows.primary().cursor_position().unwrap();
        let cursor = Vec3::new(
            cursor.x - level.area.x / 2.0,
            cursor.y - level.area.y / 2.0,
            0.0,
        );
        for (entity, transform, velocity) in query.iter() {
            let force =
                generate_seek_force(transform.translation, cursor, velocity.0, &level, &conf);
            if force.length_squared() > 0.0 {
                let mut forces = steering_force_query.get_mut(entity).unwrap();
                forces.0.push(SteeringForce::Seek(force.normalize()));
            }
        }
    }
}

fn generate_seek_force(
    origin: Vec3,
    target: Vec3,
    current_velocity: Vec3,
    level: &Level,
    conf: &SteeringForceConf,
) -> Vec3 {
    let desired = shortest_path_to_target(&level, target, origin).normalize() * conf.max_speed;
    desired - current_velocity
}

fn apply_cohesion(
    level: Res<Level>,
    conf: Res<SteeringForceConf>,
    query: Query<(Entity, &Transform, &Velocity), With<Boid>>,
    mut steering_force_query: Query<(Entity, &mut SteeringForces)>,
) {
    for (target, mut forces) in steering_force_query.iter_mut() {
        let mut center_of_mass = Vec3::ZERO;
        let mut neighbours = 0;
        let (_, target_transform, target_vel) = query.get(target).unwrap();
        for (neighbour, neighbour_transform, _) in query.iter() {
            if shortest_path_to_target(
                &level,
                target_transform.translation,
                neighbour_transform.translation,
            )
            .length_squared()
                > conf.neighbourhood_size
                || target == neighbour
            {
                continue;
            }
            center_of_mass += neighbour_transform.translation;
            neighbours += 1;
        }

        if neighbours > 0 {
            let force = generate_seek_force(
                target_transform.translation,
                center_of_mass / neighbours as f32,
                target_vel.0,
                &level,
                &conf,
            );
            if force.length_squared() > 0.0 {
                forces.0.push(SteeringForce::Cohesion(force.normalize()));
            }
        }
    }
}

fn on_resize_system(mut level: ResMut<Level>, mut resize_reader: EventReader<WindowResized>) {
    for e in resize_reader.iter() {
        level.area = Vec2::new(e.width, e.height)
    }
}
