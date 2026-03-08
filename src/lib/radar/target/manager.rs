//! Shared target manager for dual radar support
//!
//! Manages ARPA targets across multiple radars, allowing targets to seamlessly
//! transition between radars based on optimal coverage (smallest range that
//! can see the target provides best resolution).

use std::{
    collections::HashMap,
    sync::{Arc, RwLock},
};

use crate::radar::GeoPosition;

use super::kalman::Polar;
use super::{
    ArpaTarget, ArpaTargetApi, Doppler, ExtendedPosition, RefreshState, TargetStatus,
    METERS_PER_DEGREE_LATITUDE,
};

/// Configuration for a radar participating in shared target management
#[derive(Debug, Clone)]
pub(crate) struct RadarTargetConfig {
    /// Unique key identifying this radar
    pub(crate) key: String,
    /// Current range in meters
    pub(crate) range_meters: u32,
    /// Pixels per meter at current range
    pub(crate) pixels_per_meter: f64,
    /// Number of spokes per revolution
    pub(crate) spokes_per_revolution: i32,
    /// Current radar position
    pub(crate) position: GeoPosition,
    /// Whether this radar is currently transmitting
    pub(crate) transmitting: bool,
}

/// A target managed by the shared target manager
#[derive(Debug, Clone)]
pub(crate) struct ManagedTarget {
    /// The underlying ARPA target
    pub(crate) target: ArpaTarget,
    /// Key of the radar currently tracking this target
    pub(crate) tracking_radar: String,
    /// Key of the radar that originally acquired this target
    pub(crate) source_radar: String,
    /// Whether this target was recently transferred from another radar
    pub(crate) transferred: bool,
    /// Last refresh timestamp
    pub(crate) last_refresh: u64,
}

/// Internal state of the target manager
struct TargetManagerState {
    /// All tracked targets, keyed by global target ID
    targets: HashMap<usize, ManagedTarget>,
    /// Next available target ID
    next_target_id: usize,
    /// Radar configurations for range selection
    radar_configs: HashMap<String, RadarTargetConfig>,
}

/// Shared target manager for coordinating targets across multiple radars
///
/// This manager maintains a single pool of targets that can be tracked by
/// any of the registered radars. Targets automatically transition to the
/// radar with the best coverage (smallest range that can see the target).
#[derive(Clone)]
pub struct SharedTargetManager {
    inner: Arc<RwLock<TargetManagerState>>,
}

impl std::fmt::Debug for SharedTargetManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.inner.read().unwrap();
        f.debug_struct("SharedTargetManager")
            .field("target_count", &state.targets.len())
            .field("radar_count", &state.radar_configs.len())
            .finish()
    }
}

impl SharedTargetManager {
    /// Create a new shared target manager
    pub(crate) fn new() -> Self {
        SharedTargetManager {
            inner: Arc::new(RwLock::new(TargetManagerState {
                targets: HashMap::new(),
                next_target_id: 1,
                radar_configs: HashMap::new(),
            })),
        }
    }

    /// Register a radar for target management
    pub(crate) fn register_radar(&self, config: RadarTargetConfig) {
        let mut state = self.inner.write().unwrap();
        log::debug!(
            "Registering radar '{}' with range {}m for target management",
            config.key,
            config.range_meters
        );
        state.radar_configs.insert(config.key.clone(), config);
    }

    /// Update radar configuration (e.g., when range changes)
    pub(crate) fn update_radar_config(&self, config: RadarTargetConfig) {
        let mut state = self.inner.write().unwrap();
        if state.radar_configs.contains_key(&config.key) {
            state.radar_configs.insert(config.key.clone(), config);
        }
    }

    /// Remove a radar from target management
    pub(crate) fn unregister_radar(&self, key: &str) {
        let mut state = self.inner.write().unwrap();
        log::debug!("Unregistering radar '{}' from target management", key);
        state.radar_configs.remove(key);

        // Mark targets tracked by this radar as needing reassignment
        for managed in state.targets.values_mut() {
            if managed.tracking_radar == key {
                managed.transferred = true;
            }
        }
    }

    /// Find the best radar for tracking a target at the given position
    ///
    /// Returns the radar with the smallest range that can still see the target,
    /// as smaller range provides better resolution.
    pub(crate) fn find_best_radar_for_target(&self, target_position: &GeoPosition) -> Option<String> {
        let state = self.inner.read().unwrap();
        self.find_best_radar_internal(&state, target_position)
    }

    fn find_best_radar_internal(
        &self,
        state: &TargetManagerState,
        target_position: &GeoPosition,
    ) -> Option<String> {
        let mut best_radar: Option<(&str, u32)> = None;

        for (key, config) in &state.radar_configs {
            if !config.transmitting {
                continue;
            }

            let distance = Self::calculate_distance(&config.position, target_position);
            // Allow some margin for target size (99% of range)
            let effective_range = config.range_meters as f64 * 0.99;

            if distance <= effective_range {
                match best_radar {
                    None => {
                        best_radar = Some((key.as_str(), config.range_meters));
                    }
                    Some((_, best_range)) if config.range_meters < best_range => {
                        best_radar = Some((key.as_str(), config.range_meters));
                    }
                    _ => {}
                }
            }
        }

        best_radar.map(|(key, _)| key.to_string())
    }

    /// Calculate distance between two positions in meters
    fn calculate_distance(pos1: &GeoPosition, pos2: &GeoPosition) -> f64 {
        const METERS_PER_NM: f64 = 1852.0;
        const MINUTES_PER_DEGREE: f64 = 60.0;

        let dlat = (pos2.lat - pos1.lat) * MINUTES_PER_DEGREE * METERS_PER_NM;
        let dlon = (pos2.lon - pos1.lon)
            * MINUTES_PER_DEGREE
            * METERS_PER_NM
            * pos1.lat.to_radians().cos();

        (dlat * dlat + dlon * dlon).sqrt()
    }

    /// Acquire a new target on the specified radar
    pub(crate) fn acquire_target(
        &self,
        radar_key: &str,
        position: ExtendedPosition,
        doppler: Doppler,
        spokes_per_revolution: usize,
        have_doppler: bool,
    ) -> usize {
        let mut state = self.inner.write().unwrap();

        let target_id = state.next_target_id;
        state.next_target_id += 1;
        if state.next_target_id >= 100000 {
            state.next_target_id = 1;
        }

        // Get radar config to calculate polar coordinates
        let radar_config = state.radar_configs.get(radar_key).cloned();

        // Get radar position from config or use default
        let radar_pos = radar_config
            .as_ref()
            .map(|c| c.position.clone())
            .unwrap_or_else(|| GeoPosition::new(0., 0.));

        let mut target = ArpaTarget::new(
            position.clone(),
            radar_pos.clone(),
            target_id,
            spokes_per_revolution,
            TargetStatus::Acquire0,
            have_doppler,
        );

        // Calculate polar coordinates if we have radar config with valid pixels_per_meter
        if let Some(config) = &radar_config {
            if config.pixels_per_meter > 0.0 {
                let polar = Self::pos2polar(
                    &position,
                    &radar_pos,
                    config.pixels_per_meter,
                    config.spokes_per_revolution as f64,
                );
                // Set contour position and expected so refresh can find the target
                target.contour.position = polar.clone();
                target.expected = polar.clone();

                log::debug!(
                    "Acquired target {} polar: angle={}, r={}, pixels_per_meter={}",
                    target_id,
                    polar.angle,
                    polar.r,
                    config.pixels_per_meter
                );
            }
        }

        let managed = ManagedTarget {
            target,
            tracking_radar: radar_key.to_string(),
            source_radar: radar_key.to_string(),
            transferred: false,
            last_refresh: position.time,
        };

        log::debug!(
            "Acquired new target {} on radar '{}' at ({}, {})",
            target_id,
            radar_key,
            position.pos.lat,
            position.pos.lon
        );

        state.targets.insert(target_id, managed);
        target_id
    }

    /// Convert lat/lon position to polar coordinates (angle in spokes, r in pixels)
    fn pos2polar(
        target: &ExtendedPosition,
        radar_pos: &GeoPosition,
        pixels_per_meter: f64,
        spokes_per_revolution: f64,
    ) -> Polar {
        use std::f64::consts::PI;

        let dif_lat = target.pos.lat - radar_pos.lat;
        let dif_lon = (target.pos.lon - radar_pos.lon) * radar_pos.lat.to_radians().cos();
        let r = ((dif_lat * dif_lat + dif_lon * dif_lon).sqrt()
            * METERS_PER_DEGREE_LATITUDE
            * pixels_per_meter
            + 1.) as i32;
        let mut angle = f64::atan2(dif_lon, dif_lat) * spokes_per_revolution / (2. * PI) + 1.;
        if angle < 0. {
            angle += spokes_per_revolution;
        }
        Polar::new(angle as i32, r, target.time)
    }

    /// Add an already-constructed target to the shared manager
    pub(crate) fn add_target(&self, target_id: usize, target: ArpaTarget, radar_key: &str) {
        let mut state = self.inner.write().unwrap();

        let managed = ManagedTarget {
            target,
            tracking_radar: radar_key.to_string(),
            source_radar: radar_key.to_string(),
            transferred: false,
            last_refresh: 0,
        };

        log::debug!(
            "Added target {} to shared manager for radar '{}'",
            target_id,
            radar_key
        );

        state.targets.insert(target_id, managed);
    }

    /// Get all targets currently assigned to a specific radar
    pub(crate) fn get_targets_for_radar(&self, radar_key: &str) -> Vec<(usize, ArpaTarget)> {
        let state = self.inner.read().unwrap();
        state
            .targets
            .iter()
            .filter(|(_, managed)| managed.tracking_radar == radar_key)
            .map(|(id, managed)| (*id, managed.target.clone()))
            .collect()
    }

    /// Get a specific target by ID
    pub(crate) fn get_target(&self, target_id: usize) -> Option<ManagedTarget> {
        let state = self.inner.read().unwrap();
        state.targets.get(&target_id).cloned()
    }

    /// Update a target after refresh
    pub(crate) fn update_target(&self, target_id: usize, target: ArpaTarget, refresh_time: u64) {
        let mut state = self.inner.write().unwrap();
        if let Some(managed) = state.targets.get_mut(&target_id) {
            managed.target = target;
            managed.last_refresh = refresh_time;
            managed.transferred = false;
        }
    }

    /// Transfer a target to a different radar
    pub(crate) fn transfer_target(&self, target_id: usize, new_radar: &str) -> bool {
        let mut state = self.inner.write().unwrap();
        if let Some(managed) = state.targets.get_mut(&target_id) {
            if managed.tracking_radar != new_radar {
                log::debug!(
                    "Transferring target {} from '{}' to '{}'",
                    target_id,
                    managed.tracking_radar,
                    new_radar
                );
                managed.tracking_radar = new_radar.to_string();
                managed.transferred = true;
                return true;
            }
        }
        false
    }

    /// Evaluate all targets and transfer them to better radars if available
    pub(crate) fn evaluate_radar_transfers(&self) {
        let mut state = self.inner.write().unwrap();

        let mut transfers: Vec<(usize, String)> = Vec::new();

        for (target_id, managed) in &state.targets {
            if managed.target.status == TargetStatus::Lost {
                continue;
            }

            if let Some(best_radar) =
                self.find_best_radar_internal(&state, &managed.target.position.pos)
            {
                if best_radar != managed.tracking_radar {
                    transfers.push((*target_id, best_radar));
                }
            }
        }

        // Apply transfers
        for (target_id, new_radar) in transfers {
            if let Some(managed) = state.targets.get_mut(&target_id) {
                log::debug!(
                    "Auto-transferring target {} from '{}' to '{}'",
                    target_id,
                    managed.tracking_radar,
                    new_radar
                );
                managed.tracking_radar = new_radar;
                managed.transferred = true;
            }
        }
    }

    /// Delete a target by ID
    pub(crate) fn delete_target(&self, target_id: usize) -> bool {
        let mut state = self.inner.write().unwrap();
        state.targets.remove(&target_id).is_some()
    }

    /// Delete the target closest to the given position
    pub(crate) fn delete_target_near(&self, position: &GeoPosition) -> Option<usize> {
        let mut state = self.inner.write().unwrap();

        let mut closest: Option<(usize, f64)> = None;

        for (id, managed) in &state.targets {
            if managed.target.status == TargetStatus::Lost {
                continue;
            }

            let distance = Self::calculate_distance(&managed.target.position.pos, position);
            match closest {
                None => closest = Some((*id, distance)),
                Some((_, min_dist)) if distance < min_dist => closest = Some((*id, distance)),
                _ => {}
            }
        }

        if let Some((id, dist)) = closest {
            if dist < 1000.0 {
                // Within 1km
                state.targets.remove(&id);
                return Some(id);
            }
        }

        None
    }

    /// Delete all targets
    pub(crate) fn delete_all_targets(&self) {
        let mut state = self.inner.write().unwrap();
        state.targets.clear();
    }

    /// Get all target IDs
    pub(crate) fn get_all_target_ids(&self) -> Vec<usize> {
        let state = self.inner.read().unwrap();
        state.targets.keys().copied().collect()
    }

    /// Clean up lost targets and reset refresh state for the next cycle
    pub(crate) fn cleanup_lost_targets(&self) {
        let mut state = self.inner.write().unwrap();
        state
            .targets
            .retain(|_, managed| managed.target.status != TargetStatus::Lost);
        // Reset refresh state for all remaining targets so they can be refreshed next cycle
        for managed in state.targets.values_mut() {
            managed.target.refreshed = RefreshState::NotFound;
        }
    }

    /// Get the number of active targets
    pub(crate) fn target_count(&self) -> usize {
        let state = self.inner.read().unwrap();
        state.targets.len()
    }

    /// Get the number of registered radars
    pub(crate) fn radar_count(&self) -> usize {
        let state = self.inner.read().unwrap();
        state.radar_configs.len()
    }

    /// Check if dual radar mode is active (2+ transmitting radars)
    pub(crate) fn is_dual_radar_active(&self) -> bool {
        let state = self.inner.read().unwrap();
        state
            .radar_configs
            .values()
            .filter(|c| c.transmitting)
            .count()
            >= 2
    }

    /// Get radar keys sorted by range (short to long)
    pub(crate) fn get_radars_by_range(&self) -> Vec<String> {
        let state = self.inner.read().unwrap();
        let mut radars: Vec<_> = state
            .radar_configs
            .values()
            .filter(|c| c.transmitting)
            .collect();
        radars.sort_by_key(|c| c.range_meters);
        radars.iter().map(|c| c.key.clone()).collect()
    }

    /// Get the short range radar key (smallest range)
    pub(crate) fn get_short_range_radar(&self) -> Option<String> {
        self.get_radars_by_range().first().cloned()
    }

    /// Get the long range radar key (largest range)
    pub(crate) fn get_long_range_radar(&self) -> Option<String> {
        self.get_radars_by_range().last().cloned()
    }

    /// Get all targets grouped by their tracking radar as API representations
    /// Returns a map of radar_key -> Vec<(target_id, ArpaTargetApi)>
    pub fn get_all_targets_by_radar(
        &self,
        radar_position: &GeoPosition,
    ) -> std::collections::HashMap<String, Vec<(usize, ArpaTargetApi)>> {
        let state = self.inner.read().unwrap();
        let mut result: std::collections::HashMap<String, Vec<(usize, ArpaTargetApi)>> =
            std::collections::HashMap::new();

        for (id, managed) in &state.targets {
            // Only include targets that should be broadcast (ACQUIRE3 or ACTIVE)
            if managed.target.should_broadcast() {
                result
                    .entry(managed.tracking_radar.clone())
                    .or_default()
                    .push((*id, managed.target.to_api(radar_position)));
            }
        }

        result
    }
}

impl Default for SharedTargetManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_config(key: &str, range: u32, lat: f64, lon: f64) -> RadarTargetConfig {
        RadarTargetConfig {
            key: key.to_string(),
            range_meters: range,
            pixels_per_meter: 1.0,
            spokes_per_revolution: 2048,
            position: GeoPosition::new(lat, lon),
            transmitting: true,
        }
    }

    #[test]
    fn test_find_best_radar_prefers_smaller_range() {
        let manager = SharedTargetManager::new();

        // Register two radars at same position with different ranges
        manager.register_radar(create_test_config("radar_a", 5000, 52.0, 4.0));
        manager.register_radar(create_test_config("radar_b", 10000, 52.0, 4.0));

        // Target within range of both
        let target_pos = GeoPosition::new(52.01, 4.01);
        let best = manager.find_best_radar_for_target(&target_pos);

        assert_eq!(best, Some("radar_a".to_string()));
    }

    #[test]
    fn test_find_best_radar_falls_back_to_longer_range() {
        let manager = SharedTargetManager::new();

        // Register two radars at same position with different ranges
        manager.register_radar(create_test_config("radar_a", 2000, 52.0, 4.0));
        manager.register_radar(create_test_config("radar_b", 10000, 52.0, 4.0));

        // Target outside short range but within long range
        let target_pos = GeoPosition::new(52.05, 4.05); // ~7km away
        let best = manager.find_best_radar_for_target(&target_pos);

        assert_eq!(best, Some("radar_b".to_string()));
    }

    #[test]
    fn test_find_best_radar_returns_none_if_out_of_range() {
        let manager = SharedTargetManager::new();

        manager.register_radar(create_test_config("radar_a", 1000, 52.0, 4.0));

        // Target far away
        let target_pos = GeoPosition::new(53.0, 5.0); // ~100km away
        let best = manager.find_best_radar_for_target(&target_pos);

        assert_eq!(best, None);
    }

    #[test]
    fn test_dual_radar_active() {
        let manager = SharedTargetManager::new();

        assert!(!manager.is_dual_radar_active());

        manager.register_radar(create_test_config("radar_a", 5000, 52.0, 4.0));
        assert!(!manager.is_dual_radar_active());

        manager.register_radar(create_test_config("radar_b", 10000, 52.0, 4.0));
        assert!(manager.is_dual_radar_active());
    }
}
