use std::collections::{HashMap, HashSet};

use bevy::{math::Vec3Swizzles, prelude::*, sprite::MaterialMesh2dBundle, window::WindowResized};
use bevy_inspector_egui::{Inspectable, InspectorPlugin};
use rand::prelude::*;

#[derive(Debug)]
struct SpatialMap {
    width: usize,
    height: usize,
    offset_x: f32,
    offset_y: f32,
    cell_width: f32,
    cell_height: f32,
    entries: Vec<Option<HashSet<Entity>>>,
    lookup_cache: HashMap<i32, HashMap<(usize, usize), HashSet<Entity>>>,
}

struct SpatialMapResource {
    map: SpatialMap,
}

impl Default for SpatialMapResource {
    fn default() -> Self {
        Self {
            map: SpatialMap {
                width: 0,
                height: 0,
                offset_x: 0.0,
                offset_y: 0.0,
                entries: Vec::new(),
                cell_width: 1.0,
                cell_height: 1.0,
                lookup_cache: HashMap::new(),
            },
        }
    }
}

impl SpatialMap {
    fn new(
        width: usize,
        height: usize,
        offset_x: f32,
        offset_y: f32,
        world_width: f32,
        world_height: f32,
    ) -> Self {
        let entries = vec![None; width * height];
        Self {
            width,
            height,
            offset_x,
            offset_y,
            entries,
            cell_width: world_width / width as f32,
            cell_height: world_height / height as f32,
            lookup_cache: HashMap::new(),
        }
    }

    fn load_entity(&mut self, entity: Entity, position: &Vec3) {
        let index = self.get_index_from_world_coordinates(position.x, position.y);
        if let Some(index) = index {
            let entry = self.entries.get_mut(index).unwrap();
            if let Some(set) = entry {
                set.insert(entity);
            } else {
                let mut set = HashSet::new();
                set.insert(entity);
                self.entries.insert(index, Some(set));
            }
        } else {
            println!("Could not load entity with position {:?}", position);
        }
    }

    fn get_index_from_world_coordinates(&self, x: f32, y: f32) -> Option<usize> {
        if let Some((x, y)) = self.get_local_coordinates_from_world_coodinates(x, y) {
            Some(self.get_index_from_local_coordinates(x, y))
        } else {
            None
        }
    }

    fn get_local_coordinates_from_world_coodinates(
        &self,
        x: f32,
        y: f32,
    ) -> Option<(usize, usize)> {
        let x = (x + self.offset_x) / self.cell_width;
        let y = (y + self.offset_y) / self.cell_height;

        if x < 0.0 || y < 0.0 || x >= self.width as f32 || y >= self.height as f32 {
            None
        } else {
            Some((x as usize, y as usize))
        }
    }

    fn get_index_from_local_coordinates(&self, x: usize, y: usize) -> usize {
        x + y * self.width
    }

    fn get_all_entities_in_radius(
        &mut self,
        origin: Vec3,
        spatial_radius: i32,
        allow_wrapping: bool,
    ) -> HashSet<Entity> {
        if let Some((origin_x, origin_y)) =
            self.get_local_coordinates_from_world_coodinates(origin.x, origin.y)
        {
            if let Some(inner) = self.lookup_cache.get_mut(&spatial_radius) {
                if let Some(result) = inner.get(&(origin_x, origin_y)) {
                    return result.clone();
                }
            } else {
                self.lookup_cache.insert(spatial_radius, HashMap::new());
            }

            let mut set = HashSet::new();
            for offset_x in -spatial_radius..spatial_radius {
                let (origin_x, origin_y) = (origin_x as i32, origin_y as i32);
                if let Some(x) = correct_bounds(origin_x + offset_x, 0, self.width as i32, true) {
                    for offset_y in -spatial_radius..spatial_radius {
                        if let Some(y) = correct_bounds(
                            origin_y + offset_y,
                            0,
                            self.height as i32,
                            allow_wrapping,
                        ) {
                            let index =
                                self.get_index_from_local_coordinates(x as usize, y as usize);
                            if let Some(entities) = self.entries.get(index).unwrap() {
                                set.extend(entities.iter())
                            }
                        }
                    }
                }
            }
            self.lookup_cache
                .get_mut(&spatial_radius)
                .unwrap()
                .insert((origin_x, origin_y), set.clone());
            set
        } else {
            HashSet::new()
        }
    }
}

fn correct_bounds(
    mut value: i32,
    lower_bound: i32,
    upper_bound: i32,
    allow_wrapping: bool,
) -> Option<i32> {
    if value < lower_bound || value >= upper_bound {
        if allow_wrapping {
            value = if value < lower_bound {
                upper_bound - lower_bound + value
            } else {
                value - upper_bound + lower_bound
            };
        } else {
            return None;
        }
    }
    Some(value)
}

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
    #[inspectable(min = 10.0, max = 10000.0)]
    max_speed: f32,
    neighbourhood_size: f32,
    steering_force_tweaker: f32,
    max_steering_force: f32,
}

impl SteeringForceConf {
    fn get_neighbourhood(&self) -> f32 {
        self.neighbourhood_size * self.neighbourhood_size
    }
}

impl Default for SteeringForceConf {
    fn default() -> Self {
        Self {
            alignment: 1.0,
            separation: 2.0,
            cohesion: 2.0,
            seek: 10.0,
            max_speed: 1000.0,
            neighbourhood_size: 1000.0,
            steering_force_tweaker: 200.0,
            max_steering_force: 4.0,
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
        .insert_resource(SpatialMapResource::default())
        .add_system(update_position)
        .add_system(on_resize_system)
        .add_startup_system(setup)
        .add_startup_system(spawn_boids)
        .add_system(
            load_spatial_map
                .before(apply_separation)
                .before(apply_alignment)
                .before(apply_cohesion)
                .before(apply_seek)
                .before(apply_flee),
        )
        .add_system(apply_separation)
        .add_system(apply_alignment)
        .add_system(apply_cohesion)
        .add_system(apply_seek)
        .add_system(apply_flee)
        .add_system(
            apply_steering_force
                .after(apply_separation)
                .after(apply_alignment)
                .after(apply_cohesion)
                .after(apply_seek)
                .after(apply_flee),
        )
        .run();
}

fn setup(mut commands: Commands, mut level: ResMut<Level>, windows: ResMut<Windows>) {
    commands.spawn().insert_bundle(Camera2dBundle::default());
    level.area = Vec2::new(windows.primary().width(), windows.primary().height());
}

fn load_spatial_map(
    mut res: ResMut<SpatialMapResource>,
    query: Query<(Entity, &Transform), With<Boid>>,
    level: Res<Level>,
) {
    // TODO: We should only remake the map if there was a size change, otherwise
    // just clear.
    let mut map = SpatialMap::new(
        10,
        10,
        level.area.x / 2.0,
        level.area.y / 2.0,
        level.area.x,
        level.area.y,
    );
    for (entity, transform) in query.iter() {
        map.load_entity(entity, &transform.translation);
    }
    res.map = map;
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
    for i in 0..2000 {
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
    mut spatial_map: ResMut<SpatialMapResource>,
    mut steering_force_query: Query<(Entity, &mut SteeringForces)>,
) {
    for (target, mut forces) in steering_force_query.iter_mut() {
        let mut average_heading = Vec3::ZERO;
        let (_, target_transform, target_vel) = velocity_query.get(target).unwrap();
        let neighbours =
            spatial_map
                .map
                .get_all_entities_in_radius(target_transform.translation, 2, true);
        let mut neighbour_count = neighbours.len();
        for neighbour in neighbours.into_iter() {
            if target == neighbour {
                neighbour_count -= 1;
                continue;
            }
            let (_, _, neighbour_vel) = velocity_query.get(neighbour).unwrap();
            average_heading += neighbour_vel.heading();
        }

        if neighbour_count > 0 {
            let corrected_heading = average_heading - target_vel.0;
            let normalized_heading = corrected_heading / (neighbour_count as f32);

            forces.0.push(SteeringForce::Alignment(normalized_heading));
        }
    }
}

fn apply_separation(
    level: Res<Level>,
    conf: Res<SteeringForceConf>,
    position_query: Query<(Entity, &Transform), With<Boid>>,
    mut spatial_map: ResMut<SpatialMapResource>,
    mut steering_force_query: Query<&mut SteeringForces>,
) {
    for (target, target_transform) in position_query.iter() {
        let mut steering_force = Vec3::ZERO;
        let neighbours =
            spatial_map
                .map
                .get_all_entities_in_radius(target_transform.translation, 4, true);

        for neighbour in neighbours.into_iter() {
            let (_, neighbour_transform) = position_query.get(neighbour).unwrap();
            let distance = shortest_path_to_target(
                &level,
                target_transform.translation,
                neighbour_transform.translation,
            );
            if distance.length_squared() == 0.0 || target == neighbour {
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
    if buttons.pressed(MouseButton::Left) && false {
        if let Some(cursor) = windows.primary().cursor_position() {
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
}

fn apply_flee(
    buttons: Res<Input<MouseButton>>,
    level: Res<Level>,
    windows: Res<Windows>,
    conf: Res<SteeringForceConf>,
    query: Query<(Entity, &Transform, &Velocity), With<Boid>>,
    mut steering_force_query: Query<&mut SteeringForces>,
) {
    if buttons.pressed(MouseButton::Right) {
        if let Some(cursor) = windows.primary().cursor_position() {
            let cursor = Vec3::new(
                cursor.x - level.area.x / 2.0,
                cursor.y - level.area.y / 2.0,
                0.0,
            );
            for (entity, transform, velocity) in query.iter() {
                let force =
                    generate_seek_force(cursor, transform.translation, velocity.0, &level, &conf);
                if force.length_squared() > 0.0 {
                    let mut forces = steering_force_query.get_mut(entity).unwrap();
                    forces.0.push(SteeringForce::Seek(force.normalize()));
                }
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
    mut spatial_map: ResMut<SpatialMapResource>,
    mut steering_force_query: Query<(Entity, &mut SteeringForces)>,
) {
    for (target, mut forces) in steering_force_query.iter_mut() {
        let mut center_of_mass = Vec3::ZERO;

        let (_, target_transform, target_vel) = query.get(target).unwrap();
        let neighbours =
            spatial_map
                .map
                .get_all_entities_in_radius(target_transform.translation, 2, true);
        let mut neighbour_count = neighbours.len();
        for neighbour in neighbours.into_iter() {
            if target == neighbour {
                neighbour_count -= 1;
                continue;
            }
            let (_, neighbour_transform, _) = query.get(neighbour).unwrap();
            center_of_mass += neighbour_transform.translation;
        }

        if neighbour_count > 0 {
            let force = generate_seek_force(
                target_transform.translation,
                center_of_mass / neighbour_count as f32,
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
