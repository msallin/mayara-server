use std::f64::consts::PI;

use crate::radar::GeoPosition;

// Constants for east-moving targets simulation
const TARGET_SPEED_KNOTS: f64 = 5.0;
const TARGET_DISTANCE_SOUTH: f64 = 500.0; // meters
const TARGET_SPACING: f64 = 500.0; // meters between targets
const NUM_TARGETS: usize = 5;

// Crossing targets: boats coming from north, heading south at 10 knots
const CROSSING_SPEED_KNOTS: f64 = 10.0;
const CROSSING_SPACING: f64 = 500.0; // meters between crossing boats
const NUM_CROSSING_TARGETS: usize = 3;

// Land area dimensions - 700m south, 600m west of radar, oriented east-west
const LAND_DISTANCE_SOUTH: f64 = 700.0; // meters south
const LAND_DISTANCE_WEST: f64 = 600.0; // meters west
const LAND_WIDTH: f64 = 200.0; // meters (north-south extent)
const LAND_LENGTH: f64 = 1000.0; // meters (east-west extent)
const LAND_ORIENTATION: f64 = 90.0; // degrees from North (pointing East-West)

// Conversion constants
const KNOTS_TO_MS: f64 = 1852.0 / 3600.0; // 1 knot = 1852m/h = 0.5144 m/s
const DEG_TO_RAD: f64 = PI / 180.0;

/// A moving target (boat)
#[derive(Clone, Debug)]
pub struct Target {
    /// Current position
    pub position: GeoPosition,
    /// Heading in degrees (0 = North, 90 = East)
    pub heading: f64,
    /// Speed in m/s
    pub speed: f64,
}

impl Target {
    fn new(position: GeoPosition, heading: f64, speed_knots: f64) -> Self {
        Target {
            position,
            heading,
            speed: speed_knots * KNOTS_TO_MS,
        }
    }

    fn update(&mut self, elapsed_secs: f64) {
        let distance = self.speed * elapsed_secs;
        let heading_rad = self.heading * DEG_TO_RAD;
        self.position = self.position.position_from_bearing(heading_rad, distance);
    }
}

/// A static buoy (small radar target)
#[derive(Clone, Debug)]
pub struct Buoy {
    /// Position of the buoy
    pub position: GeoPosition,
    /// Radar return radius in meters (typically small, ~5m)
    pub radius: f64,
}

impl Buoy {
    fn new(position: GeoPosition, radius: f64) -> Self {
        Buoy { position, radius }
    }
}

/// Land area (stationary oblong)
#[derive(Clone, Debug)]
pub struct LandArea {
    /// Center position
    pub center: GeoPosition,
    /// Half-width (perpendicular to orientation)
    pub half_width: f64,
    /// Half-length (along orientation)
    pub half_length: f64,
    /// Orientation in radians (angle of the long axis from North)
    pub orientation_rad: f64,
}

impl LandArea {
    fn new(center: GeoPosition, width: f64, length: f64, orientation_deg: f64) -> Self {
        LandArea {
            center,
            half_width: width / 2.0,
            half_length: length / 2.0,
            orientation_rad: orientation_deg * DEG_TO_RAD,
        }
    }

    /// Check if a point is inside the land area
    fn contains(&self, point: &GeoPosition) -> bool {
        // Calculate distance and bearing from center to point
        let (distance, bearing) = distance_and_bearing(&self.center, point);

        // Rotate the bearing by the negative of orientation to align with local coordinates
        let local_angle = bearing - self.orientation_rad;

        // Calculate local x, y coordinates
        let local_x = distance * local_angle.sin(); // perpendicular to long axis
        let local_y = distance * local_angle.cos(); // along long axis

        // Check if within the oblong bounds
        local_x.abs() <= self.half_width && local_y.abs() <= self.half_length
    }
}

/// The simulated world
pub struct EmulatorWorld {
    /// Land area (fixed position)
    pub land: LandArea,
    /// Moving targets (east-moving boats)
    pub targets: Vec<Target>,
    /// Crossing targets (north-to-south boats)
    pub crossing_targets: Vec<Target>,
    /// Static buoys
    pub buoys: Vec<Buoy>,
}

impl EmulatorWorld {
    pub fn new(initial_boat_pos: GeoPosition) -> Self {
        // Create land area 700m south, directly south of initial position
        let south_bearing = 180.0 * DEG_TO_RAD;
        let west_bearing = 270.0 * DEG_TO_RAD;
        let land_south = initial_boat_pos.position_from_bearing(south_bearing, LAND_DISTANCE_SOUTH);
        let land_center = if LAND_DISTANCE_WEST > 0.0 {
            land_south.position_from_bearing(west_bearing, LAND_DISTANCE_WEST)
        } else {
            land_south
        };
        // Orientation: long axis points East-West (90 degrees from North)
        let land = LandArea::new(land_center, LAND_WIDTH, LAND_LENGTH, LAND_ORIENTATION);

        // Create targets 500m south of initial boat position, moving East
        // They start at various positions West of the boat
        let mut targets = Vec::with_capacity(NUM_TARGETS);
        let south_bearing = 180.0 * DEG_TO_RAD; // South
        let west_bearing = 270.0 * DEG_TO_RAD; // West

        // Base position: 500m south of boat
        let base_pos = initial_boat_pos.position_from_bearing(south_bearing, TARGET_DISTANCE_SOUTH);

        for i in 0..NUM_TARGETS {
            // Offset West by i * TARGET_SPACING
            let offset = i as f64 * TARGET_SPACING;
            let target_pos = base_pos.position_from_bearing(west_bearing, offset);
            // Moving East at 5 knots
            targets.push(Target::new(target_pos, 90.0, TARGET_SPEED_KNOTS));
        }

        // Create crossing targets: boats coming from North, heading South at 10 knots
        // The first one passes 30m behind (west of) the front east-moving boat
        // Front east-moving boat is at base_pos (500m south of initial_boat_pos)
        let mut crossing_targets = Vec::with_capacity(NUM_CROSSING_TARGETS);

        // Calculate crossing point: where the first crossing boat will pass
        // It should pass 30m behind (west of) the front east-moving boat
        // Place crossing boats 600m north of the east-moving boats' track
        let crossing_start_distance_north = 600.0; // meters north of east-boat track
        let behind_distance = 30.0; // meters behind (west of) front boat

        // Position where crossing will happen: 30m west of front east-boat, at same latitude
        let crossing_point = base_pos.position_from_bearing(west_bearing, behind_distance);

        // Starting position for first crossing boat: 400m north of crossing point
        let north_bearing = 0.0 * DEG_TO_RAD; // North
        let first_crossing_start =
            crossing_point.position_from_bearing(north_bearing, crossing_start_distance_north);

        for i in 0..NUM_CROSSING_TARGETS {
            // Offset east by i * CROSSING_SPACING so they form a line
            let offset = i as f64 * CROSSING_SPACING;
            let east_bearing = 90.0 * DEG_TO_RAD;
            let crossing_pos = first_crossing_start.position_from_bearing(east_bearing, offset);
            // Moving South at 10 knots
            crossing_targets.push(Target::new(crossing_pos, 180.0, CROSSING_SPEED_KNOTS));
        }

        // Create a static buoy near the east-moving boats' path
        // Place it 10m south of their track, 200m east of the front boat
        // This is at least 150m away from where the crossing happens (30m west)
        let buoy_east_offset = 200.0; // meters east of front boat
        let buoy_south_offset = 10.0; // meters south of track
        let east_bearing = 90.0 * DEG_TO_RAD;
        let buoy_base = base_pos.position_from_bearing(east_bearing, buoy_east_offset);
        let buoy_pos = buoy_base.position_from_bearing(south_bearing, buoy_south_offset);
        let buoys = vec![Buoy::new(buoy_pos, 5.0)]; // 5m radius buoy

        EmulatorWorld {
            land,
            targets,
            crossing_targets,
            buoys,
        }
    }

    /// Update all moving objects based on elapsed time
    pub fn update(&mut self, elapsed_secs: f64) {
        for target in &mut self.targets {
            target.update(elapsed_secs);
        }
        for target in &mut self.crossing_targets {
            target.update(elapsed_secs);
        }
    }

    /// Get the radar return intensity at a given position
    /// Returns 0-15 intensity value (0 = no return, 15 = strongest)
    pub fn get_intensity(&self, boat_pos: &GeoPosition, bearing_rad: f64, distance: f64) -> u8 {
        // Calculate the world position at this bearing/distance from boat
        let point = boat_pos.position_from_bearing(bearing_rad, distance);

        // Check land
        if self.land.contains(&point) {
            return 14; // Strong return for land
        }

        // Check targets - they appear as point targets with some spread
        const TARGET_RADIUS: f64 = 30.0; // meters - radar target size
        for target in &self.targets {
            let target_distance = distance_between(&point, &target.position);
            if target_distance < TARGET_RADIUS {
                // Intensity decreases with distance from target center
                let intensity = 15.0 - (target_distance / TARGET_RADIUS) * 5.0;
                return intensity.max(13.0) as u8;
            }
        }

        // Check crossing targets
        for target in &self.crossing_targets {
            let target_distance = distance_between(&point, &target.position);
            if target_distance < TARGET_RADIUS {
                let intensity = 15.0 - (target_distance / TARGET_RADIUS) * 5.0;
                return intensity.max(13.0) as u8;
            }
        }

        // Check buoys (smaller radar return)
        for buoy in &self.buoys {
            let buoy_distance = distance_between(&point, &buoy.position);
            if buoy_distance < buoy.radius {
                // Buoys give a moderate return
                return 10;
            }
        }

        0 // No return
    }
}

/// Calculate distance in meters between two positions
fn distance_between(from: &GeoPosition, to: &GeoPosition) -> f64 {
    const EARTH_RADIUS: f64 = 6_371_000.0; // meters

    let lat1 = from.lat().to_radians();
    let lat2 = to.lat().to_radians();
    let delta_lat = (to.lat() - from.lat()).to_radians();
    let delta_lon = (to.lon() - from.lon()).to_radians();

    let a =
        (delta_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS * c
}

/// Calculate distance and bearing from one position to another
/// Returns (distance in meters, bearing in radians)
fn distance_and_bearing(from: &GeoPosition, to: &GeoPosition) -> (f64, f64) {
    let distance = distance_between(from, to);

    let lat1 = from.lat().to_radians();
    let lat2 = to.lat().to_radians();
    let delta_lon = (to.lon() - from.lon()).to_radians();

    let y = delta_lon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * delta_lon.cos();
    let bearing = y.atan2(x);

    // Normalize bearing to [0, 2*PI)
    let bearing = (bearing + 2.0 * PI) % (2.0 * PI);

    (distance, bearing)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_land_area_contains() {
        let center = GeoPosition::new(53.2, 5.3);
        let land = LandArea::new(center, 200.0, 1000.0, 45.0);

        // Center should be inside
        assert!(land.contains(&center));

        // Point far away should be outside
        let far = GeoPosition::new(54.0, 6.0);
        assert!(!land.contains(&far));
    }

    #[test]
    fn test_target_movement() {
        let pos = GeoPosition::new(53.0, 5.0);
        let mut target = Target::new(pos, 90.0, 5.0); // Moving East at 5 knots

        // After 1 hour, should move ~9.26 km East
        target.update(3600.0);

        // Longitude should increase (moving East)
        assert!(target.position.lon() > 5.0);
    }

    #[test]
    fn test_distance_between() {
        let p1 = GeoPosition::new(53.0, 5.0);
        let p2 = GeoPosition::new(53.0, 5.01); // Small offset East

        let dist = distance_between(&p1, &p2);
        // At 53N, 0.01 degrees longitude is about 670m
        assert!(dist > 600.0 && dist < 700.0);
    }
}
