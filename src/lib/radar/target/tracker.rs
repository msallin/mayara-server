//! Target tracking for radar blob detection.
//!
//! This module tracks detected blobs across radar sweeps, maintaining
//! active (confirmed) and acquiring (potential) target lists.

use std::collections::HashMap;
use std::f64::consts::PI;

use super::kalman::KalmanFilter;
use super::{METERS_PER_DEGREE_LATITUDE, meters_per_degree_longitude};
use crate::radar::GeoPosition;

/// Time in milliseconds before an acquiring target is marked as lost
const ACQUIRING_LOST_TIMEOUT_MS: u64 = 10_000;

/// Time in milliseconds before a tracking target is marked as lost
const TRACKING_LOST_TIMEOUT_MS: u64 = 20_000;

/// Time in milliseconds before a stationary target is marked as lost
/// Stationary targets (buoys, anchored vessels) get extended timeout because
/// they may temporarily merge with passing targets and need more time to reappear
const STATIONARY_LOST_TIMEOUT_MS: u64 = 60_000;

/// Time in milliseconds after lost before a target is deleted
const DELETE_TIMEOUT_MS: u64 = 30_000;

/// Time in milliseconds after lost before a stationary target is deleted
const STATIONARY_DELETE_TIMEOUT_MS: u64 = 120_000;

/// Speed threshold (m/s) below which a target is considered stationary
/// 0.5 m/s = ~1 knot - accounts for GPS drift and minor movement
const STATIONARY_SPEED_THRESHOLD: f64 = 0.5;

/// Minimum number of updates before a target can be considered stationary
/// Prevents false positives from slow-starting tracks
const MIN_UPDATES_FOR_STATIONARY: u32 = 5;

/// Maximum distance (meters) for matching a blob to an active target
/// This prevents two distant targets from being conflated even if
/// the Kalman uncertainty grows large due to missed updates
const MAX_MATCH_DISTANCE_M: f64 = 150.0;

/// Extended match distance for fast targets (>10 knots / ~5 m/s)
/// Fast maneuvering targets need wider search radius
const MAX_MATCH_DISTANCE_FAST_M: f64 = 300.0;

/// Speed threshold for "fast" targets that get extended matching (m/s)
/// 10 knots = ~5.14 m/s
const FAST_TARGET_SPEED_MS: f64 = 5.0;

/// Update count threshold below which we use forced position override
/// Radar_pi uses status < 8, we use update_count < 8 (roughly equivalent)
const FORCED_POSITION_UPDATE_THRESHOLD: u32 = 8;

/// Maximum allowed turn angle (degrees) for high-speed targets
/// Targets appearing to turn more than this at speed are rejected as false matches
const MAX_TURN_ANGLE_DEG: f64 = 130.0;

/// Speed threshold (m/s) above which turn rejection is applied
const TURN_REJECTION_SPEED_MS: f64 = 5.0;

/// Status of a tracked target
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TargetStatus {
    /// Target is being acquired (first sighting, no confirmed motion yet)
    Acquiring,
    /// Target is actively being tracked (confirmed motion)
    Tracking,
    /// Target has not been seen for timeout period
    Lost,
}

impl TargetStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            TargetStatus::Acquiring => "acquiring",
            TargetStatus::Tracking => "tracking",
            TargetStatus::Lost => "lost",
        }
    }
}

/// How a target candidate was detected
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CandidateSource {
    /// Candidate detected in a guard zone (automatic acquisition)
    GuardZone(u8),
    /// Candidate is a Doppler-colored target
    Doppler,
    /// Candidate detected anywhere (only matches existing targets)
    Anywhere,
}

/// A target candidate from blob detection
#[derive(Clone, Debug)]
pub struct TargetCandidate {
    /// Timestamp when blob was detected (millis since epoch)
    pub time: u64,
    /// Geographic position (center of blob)
    pub position: GeoPosition,
    /// Size of the target in meters
    pub size_meters: f64,
    /// Source radar key
    pub radar_key: String,
    /// Maximum target speed in m/s (from ArpaDetectMode)
    pub max_target_speed_ms: f64,
    /// How this candidate was detected
    pub source: CandidateSource,
}

/// A confirmed target being actively tracked
pub struct ActiveTarget {
    /// Unique target ID (format: "T000001" or "T1000001")
    pub id: String,
    /// Current position
    pub position: GeoPosition,
    /// Previous position (for initial COG calculation)
    prev_position: Option<GeoPosition>,
    /// Current size estimate
    pub size_meters: f64,
    /// Speed over ground (m/s), None until first update
    pub sog: Option<f64>,
    /// Course over ground (radians, 0 = North), None until first update
    pub cog: Option<f64>,
    /// Kalman filter for motion estimation
    kalman: KalmanFilter,
    /// Last update timestamp
    pub last_update: u64,
    /// Number of updates received
    pub update_count: u32,
    /// Current status (Tracking or Lost)
    pub status: TargetStatus,
    /// Whether target was manually or automatically acquired
    pub is_manual: bool,
}

impl ActiveTarget {
    fn new(id: String, candidate: &TargetCandidate) -> Self {
        Self::new_with_uncertainty(id, candidate, 20.0)
    }

    /// Create a new target with custom position uncertainty (for MARPA)
    /// MARPA targets need larger uncertainty since user click position is approximate
    fn new_with_uncertainty(id: String, candidate: &TargetCandidate, position_variance: f64) -> Self {
        let mut kalman = KalmanFilter::new();
        kalman.init_with_uncertainty(candidate.position, candidate.time, position_variance);

        // GuardZone(0) indicates manual/MARPA acquisition
        let is_manual = matches!(candidate.source, CandidateSource::GuardZone(0));

        ActiveTarget {
            id,
            position: candidate.position,
            prev_position: Some(candidate.position), // Store for first COG calculation
            size_meters: candidate.size_meters,
            sog: None, // No speed until first update
            cog: None, // No course until first update
            kalman,
            last_update: candidate.time,
            update_count: 1,
            status: TargetStatus::Acquiring,
            is_manual,
        }
    }

    /// Update target with new candidate position.
    /// Returns false if update should be rejected (implausible maneuver).
    fn update(&mut self, candidate: &TargetCandidate) -> bool {
        let delta_time = (candidate.time.saturating_sub(self.last_update)) as f64 / 1000.0;

        // Calculate direct velocity from measured positions
        let (measured_sog, measured_cog) = if delta_time > 1.0 {
            let distance = calculate_distance(&self.position, &candidate.position);
            let sog = distance / delta_time;
            let cog = calculate_bearing(&self.position, &candidate.position);
            (Some(sog), Some(cog))
        } else {
            (None, None)
        };

        // Turn rejection: reject implausible maneuvers for fast targets in early tracking
        // Based on radar_pi: turn > 130° at speed > 5 m/s for status < 5
        if self.update_count >= 2 && self.update_count < 5 {
            if let (Some(current_cog), Some(new_cog), Some(speed)) =
                (self.cog, measured_cog, measured_sog)
            {
                if speed > TURN_REJECTION_SPEED_MS {
                    let mut turn = (new_cog - current_cog).to_degrees();
                    if turn > 180.0 {
                        turn -= 360.0;
                    }
                    if turn < -180.0 {
                        turn += 360.0;
                    }
                    if turn.abs() > MAX_TURN_ANGLE_DEG {
                        log::debug!(
                            "Target {}: rejecting update - turn {:.1}° at {:.1} m/s",
                            self.id,
                            turn,
                            speed
                        );
                        return false;
                    }
                }
            }
        }

        // Update Kalman filter (always, for state tracking)
        let (kalman_sog, kalman_cog) = self.kalman.update(candidate.position, candidate.time);

        // Forced position override for early tracking phases or fast targets
        // Based on radar_pi: bypass Kalman for status 2-7 OR fast targets (>10 knots)
        // This allows rapid adaptation to course changes that Kalman can't track
        let use_forced = self.update_count < FORCED_POSITION_UPDATE_THRESHOLD
            || self.sog.map(|s| s > FAST_TARGET_SPEED_MS).unwrap_or(false);

        if use_forced && delta_time > 1.0 {
            if let (Some(m_sog), Some(m_cog)) = (measured_sog, measured_cog) {
                // Blend measured and filtered values with exponential decay
                // factor = 0.8^(update_count - 1), decreasing influence as target stabilizes
                // For fast maneuvering targets, maintain minimum factor of 0.3 to stay responsive
                let is_fast = self.sog.map(|s| s > FAST_TARGET_SPEED_MS).unwrap_or(false);
                let min_factor = if is_fast { 0.3 } else { 0.0 };
                let factor =
                    0.8_f64.powi((self.update_count.saturating_sub(1)) as i32).max(min_factor);

                let current_sog = self.sog.unwrap_or(0.0);
                let current_cog = self.cog.unwrap_or(m_cog);

                // Blend SOG
                let blended_sog = current_sog + factor * (m_sog - current_sog);

                // Blend COG (handle wraparound)
                let mut cog_diff = m_cog - current_cog;
                if cog_diff > PI {
                    cog_diff -= 2.0 * PI;
                }
                if cog_diff < -PI {
                    cog_diff += 2.0 * PI;
                }
                let mut blended_cog = current_cog + factor * cog_diff;
                if blended_cog < 0.0 {
                    blended_cog += 2.0 * PI;
                }
                if blended_cog >= 2.0 * PI {
                    blended_cog -= 2.0 * PI;
                }

                self.sog = Some(blended_sog);
                self.cog = Some(blended_cog);

                // Force the Kalman filter state to match our blended values
                // This ensures predict() returns positions consistent with our motion estimate
                self.kalman
                    .force_state(candidate.position, blended_sog, blended_cog, candidate.time);

                log::trace!(
                    "Target {}: forced override factor={:.2}, sog={:.1}->{:.1}, cog={:.1}°->{:.1}°",
                    self.id,
                    factor,
                    current_sog,
                    blended_sog,
                    current_cog.to_degrees(),
                    blended_cog.to_degrees()
                );
            }
        } else if self.update_count >= 1 {
            // Use Kalman filter estimates for stable targets
            self.sog = Some(kalman_sog);
            self.cog = Some(kalman_cog);
        }

        // On first update, just use measured values directly
        if self.update_count == 1 {
            if let (Some(m_sog), Some(m_cog)) = (measured_sog, measured_cog) {
                self.sog = Some(m_sog);
                self.cog = Some(m_cog);
            }
            self.prev_position = None;
        }

        self.position = candidate.position;
        self.size_meters = candidate.size_meters;
        self.last_update = candidate.time;
        self.update_count += 1;

        // Set to Tracking once we have COG, otherwise stay Acquiring
        if self.cog.is_some() {
            self.status = TargetStatus::Tracking;
        } else if self.status == TargetStatus::Lost {
            self.status = TargetStatus::Acquiring;
        }

        true
    }

    /// Predict position at given time using Kalman filter
    pub fn predict_position(&self, time: u64) -> GeoPosition {
        self.kalman.predict(time)
    }

    fn get_uncertainty(&self) -> f64 {
        self.kalman.get_uncertainty()
    }

    /// Check if target is considered stationary (very low speed, enough updates)
    fn is_stationary(&self) -> bool {
        self.update_count >= MIN_UPDATES_FOR_STATIONARY
            && self.sog.map(|s| s < STATIONARY_SPEED_THRESHOLD).unwrap_or(false)
    }
}

/// Result of processing a target candidate
#[derive(Debug)]
pub enum ProcessResult {
    /// Target was updated (target_id)
    Updated(String),
    /// New target was promoted from acquiring to tracking (target_id)
    Promoted(String),
    /// New target was created in acquiring status (target_id)
    NewAcquiring(String),
    /// No action taken (e.g., candidate outside guard zone didn't match existing target)
    Ignored,
}

/// Statistics for logging
#[derive(Default)]
struct TrackerStats {
    candidates_processed: u32,
    active_matches: u32,
    new_acquiring: u32,
}

/// Target tracker state
pub struct TargetTracker {
    /// Active targets (including those in Acquiring status)
    active_targets: HashMap<String, ActiveTarget>,
    /// Next target ID number
    next_id: u32,
    /// ID prefix ("T" for merged, "T1" for radar 1, etc.)
    id_prefix: String,
    /// Maximum ID number before wrap
    max_id: u32,
    /// Spokes per revolution (for revolution detection)
    spokes_per_revolution: u16,
    /// Last spoke angle seen
    last_angle: u16,
    /// Revolution counter
    revolution_count: u64,
    /// Statistics for current revolution
    stats: TrackerStats,
}

impl TargetTracker {
    /// Create a new tracker for merged mode
    pub fn new_merged(spokes_per_revolution: u16) -> Self {
        TargetTracker {
            active_targets: HashMap::new(),
            next_id: 1,
            id_prefix: "T".to_string(),
            max_id: 999999,
            spokes_per_revolution,
            last_angle: 0,
            revolution_count: 0,
            stats: TrackerStats::default(),
        }
    }

    /// Create a new tracker for per-radar mode
    pub fn new_per_radar(radar_index: usize, spokes_per_revolution: u16) -> Self {
        TargetTracker {
            active_targets: HashMap::new(),
            next_id: 1,
            id_prefix: format!("T{}", radar_index),
            max_id: 99999,
            spokes_per_revolution,
            last_angle: 0,
            revolution_count: 0,
            stats: TrackerStats::default(),
        }
    }

    /// Generate next target ID
    fn next_target_id(&mut self) -> String {
        let id = if self.id_prefix == "T" {
            format!("{}{:06}", self.id_prefix, self.next_id)
        } else {
            format!("{}{:05}", self.id_prefix, self.next_id)
        };

        self.next_id += 1;
        if self.next_id > self.max_id {
            self.next_id = 0;
        }

        id
    }

    /// Check for revolution boundary and perform cleanup
    pub fn check_revolution(&mut self, angle: u16, time: u64) {
        // Detect revolution boundary (angle wraps from high to low)
        if angle < self.last_angle
            && (self.last_angle - angle) > (self.spokes_per_revolution / 2)
        {
            self.on_revolution_complete(time);
        }
        self.last_angle = angle;
    }

    /// Handle revolution complete event
    fn on_revolution_complete(&mut self, _time: u64) {
        self.revolution_count += 1;

        // Count targets by status
        let acquiring_count = self.active_targets.values()
            .filter(|t| t.status == TargetStatus::Acquiring)
            .count();
        let tracking_count = self.active_targets.values()
            .filter(|t| t.status == TargetStatus::Tracking)
            .count();

        // Log statistics
        log::info!(
            "Revolution {}: {} targets ({} acquiring, {} tracking), {} candidates processed",
            self.revolution_count,
            self.active_targets.len(),
            acquiring_count,
            tracking_count,
            self.stats.candidates_processed,
        );

        // Reset stats
        self.stats = TrackerStats::default();
    }

    /// Check for timed out targets.
    /// Returns (deleted_ids, newly_lost_ids) - both as target IDs.
    /// Marks targets as Lost if not seen for timeout period:
    /// - Acquiring targets: 10 seconds
    /// - Tracking targets: 20 seconds
    /// - Stationary targets: 60 seconds (extended to handle temporary merging)
    /// Removes targets after delete timeout (30s normal, 120s stationary).
    pub fn check_timeouts(&mut self, current_time: u64) -> (Vec<String>, Vec<String>) {
        let mut deleted_ids = Vec::new();
        let mut lost_ids = Vec::new();

        // Check each active target
        for (id, target) in &mut self.active_targets {
            let elapsed = current_time.saturating_sub(target.last_update);
            let is_stationary = target.is_stationary();

            // Use extended delete timeout for stationary targets
            let delete_timeout = if is_stationary {
                STATIONARY_DELETE_TIMEOUT_MS
            } else {
                DELETE_TIMEOUT_MS
            };

            if elapsed >= delete_timeout {
                // Mark for deletion
                deleted_ids.push(id.clone());
                log::info!(
                    "Target {} deleted after {}s without update{}",
                    id,
                    elapsed / 1000,
                    if is_stationary { " (stationary)" } else { "" }
                );
            } else if target.status != TargetStatus::Lost {
                // Use different timeout based on current status and motion
                let lost_timeout = if is_stationary {
                    // Stationary targets get extended timeout since they may
                    // temporarily merge with passing targets
                    STATIONARY_LOST_TIMEOUT_MS
                } else {
                    match target.status {
                        TargetStatus::Acquiring => ACQUIRING_LOST_TIMEOUT_MS,
                        TargetStatus::Tracking => TRACKING_LOST_TIMEOUT_MS,
                        TargetStatus::Lost => unreachable!(),
                    }
                };

                if elapsed >= lost_timeout {
                    // Mark as lost (only add to lost_ids if status is changing)
                    target.status = TargetStatus::Lost;
                    lost_ids.push(id.clone());
                    log::info!(
                        "Target {} marked as lost after {}s without update{}",
                        id,
                        elapsed / 1000,
                        if is_stationary { " (stationary)" } else { "" }
                    );
                }
            }
        }

        // Remove deleted targets
        for id in &deleted_ids {
            self.active_targets.remove(id);
        }

        (deleted_ids, lost_ids)
    }

    /// Process a target candidate, returns what happened
    pub fn process_candidate(&mut self, candidate: TargetCandidate) -> ProcessResult {
        self.stats.candidates_processed += 1;

        // 1. Try to match against active targets (including those in Acquiring status)
        if let Some(target_id) = self.match_active_target(&candidate) {
            if let Some(target) = self.active_targets.get_mut(&target_id) {
                let was_acquiring = target.status == TargetStatus::Acquiring;

                // Update may return false if the maneuver is rejected as implausible
                if !target.update(&candidate) {
                    // Rejected - don't count as match, let it potentially create new target
                    log::debug!(
                        "Update rejected for target {} - maneuver implausible",
                        target_id
                    );
                    // Fall through to create new target if from guard zone
                } else {
                    self.stats.active_matches += 1;

                    // If target transitioned from Acquiring to Tracking, report as Promoted
                    if was_acquiring && target.status == TargetStatus::Tracking {
                        log::info!(
                            "Promoted target {} to tracking at ({:.6}, {:.6}), SOG={:.1}m/s, COG={:.1}°",
                            target_id,
                            target.position.lat(),
                            target.position.lon(),
                            target.sog.unwrap_or(0.0),
                            target.cog.map(|c| c.to_degrees()).unwrap_or(0.0)
                        );
                        return ProcessResult::Promoted(target_id);
                    }

                    log::debug!(
                        "Updated active target {} at ({:.6}, {:.6}), SOG={:.1}m/s, COG={:.1}°",
                        target_id,
                        target.position.lat(),
                        target.position.lon(),
                        target.sog.unwrap_or(0.0),
                        target.cog.map(|c| c.to_degrees()).unwrap_or(0.0)
                    );
                    return ProcessResult::Updated(target_id);
                }
            }
        }

        // 2. Only create new targets from GuardZone and Doppler candidates
        // "Anywhere" candidates are only for updating existing targets
        match candidate.source {
            CandidateSource::GuardZone(_) | CandidateSource::Doppler => {
                let target_id = self.create_acquiring_target(&candidate);
                self.stats.new_acquiring += 1;
                ProcessResult::NewAcquiring(target_id)
            }
            CandidateSource::Anywhere => {
                // Don't create target - candidate didn't match any existing target
                ProcessResult::Ignored
            }
        }
    }

    /// Try to match candidate against active targets
    /// Returns the ID of the closest matching target within threshold
    fn match_active_target(&self, candidate: &TargetCandidate) -> Option<String> {
        let mut best_match: Option<(&str, f64)> = None;

        for (id, target) in &self.active_targets {
            let predicted_pos = target.predict_position(candidate.time);
            let uncertainty = target.get_uncertainty();
            let distance = calculate_distance(&predicted_pos, &candidate.position);

            // Use extended match distance for:
            // 1. Fast targets (>5 m/s / ~10 knots) - they can maneuver more
            // 2. Early tracking phases (low update count) - less certain of motion
            // Based on radar_pi: doubles search radius for fast targets and low-status
            let is_fast = target.sog.map(|s| s > FAST_TARGET_SPEED_MS).unwrap_or(false);
            let is_early = target.update_count <= 2;
            let max_dist = if is_fast || is_early {
                MAX_MATCH_DISTANCE_FAST_M
            } else {
                MAX_MATCH_DISTANCE_M
            };

            // Match threshold is the smaller of:
            // 1. 2x Kalman uncertainty (adapts to target behavior)
            // 2. Maximum match distance (prevents conflating distant targets)
            let threshold = (uncertainty * 2.0).min(max_dist);

            log::debug!(
                "Match check: target {} predicted ({:.6}, {:.6}), candidate ({:.6}, {:.6}), distance={:.1}m, threshold={:.1}m (max={:.0}m), uncertainty={:.1}m",
                id,
                predicted_pos.lat(),
                predicted_pos.lon(),
                candidate.position.lat(),
                candidate.position.lon(),
                distance,
                threshold,
                max_dist,
                uncertainty
            );

            if distance < threshold {
                // Track only the closest match
                if best_match.map_or(true, |(_, best_dist)| distance < best_dist) {
                    best_match = Some((id.as_str(), distance));
                }
            }
        }

        best_match.map(|(id, _)| id.to_string())
    }

    /// Create a new active target in Acquiring status
    fn create_acquiring_target(&mut self, candidate: &TargetCandidate) -> String {
        let id = self.next_target_id();
        let target = ActiveTarget::new(id.clone(), candidate);

        log::info!(
            "Created acquiring target {} at ({:.6}, {:.6}), size={:.1}m",
            id,
            candidate.position.lat(),
            candidate.position.lon(),
            candidate.size_meters
        );

        self.active_targets.insert(id.clone(), target);
        id
    }

    /// Directly add a target as active (for MARPA - manual acquisition)
    /// Returns the new target ID
    pub fn add_active_target(&mut self, candidate: &TargetCandidate) -> String {
        let id = self.next_target_id();
        // MARPA targets need larger initial uncertainty since user clicks are approximate
        // Position variance of 1250 gives ~100m uncertainty (2 * sqrt(1250 + 1250))
        let target = ActiveTarget::new_with_uncertainty(id.clone(), candidate, 1250.0);

        log::info!(
            "MARPA: Created active target {} at ({:.6}, {:.6}), size={:.1}m",
            id,
            candidate.position.lat(),
            candidate.position.lon(),
            candidate.size_meters
        );

        self.active_targets.insert(id.clone(), target);
        id
    }

    /// Get all active targets
    pub fn get_active_targets(&self) -> impl Iterator<Item = &ActiveTarget> {
        self.active_targets.values()
    }

    /// Get a specific active target by ID
    pub fn get_target(&self, id: &str) -> Option<&ActiveTarget> {
        self.active_targets.get(id)
    }

    /// Get number of active targets (including those in Acquiring status)
    pub fn active_count(&self) -> usize {
        self.active_targets.len()
    }
}

/// Calculate distance between two positions in meters
fn calculate_distance(p1: &GeoPosition, p2: &GeoPosition) -> f64 {
    let dlat = (p2.lat() - p1.lat()) * METERS_PER_DEGREE_LATITUDE;
    let dlon = (p2.lon() - p1.lon()) * meters_per_degree_longitude(&p1.lat());
    (dlat * dlat + dlon * dlon).sqrt()
}

/// Calculate bearing from p1 to p2 in radians (0 = North)
fn calculate_bearing(p1: &GeoPosition, p2: &GeoPosition) -> f64 {
    let dlat = (p2.lat() - p1.lat()) * METERS_PER_DEGREE_LATITUDE;
    let dlon = (p2.lon() - p1.lon()) * meters_per_degree_longitude(&p1.lat());

    let bearing = dlon.atan2(dlat);
    if bearing < 0.0 {
        bearing + 2.0 * PI
    } else {
        bearing
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Default max speed for tests (50 knots)
    const TEST_MAX_SPEED_MS: f64 = 50.0 * 0.5144;

    fn make_candidate(lat: f64, lon: f64, time: u64) -> TargetCandidate {
        TargetCandidate {
            time,
            position: GeoPosition::new(lat, lon),
            size_meters: 30.0,
            radar_key: "test".to_string(),
            max_target_speed_ms: TEST_MAX_SPEED_MS,
            source: CandidateSource::GuardZone(1), // Default to guard zone for tests
        }
    }

    #[test]
    fn test_target_id_generation_merged() {
        let mut tracker = TargetTracker::new_merged(2048);
        assert_eq!(tracker.next_target_id(), "T000001");
        assert_eq!(tracker.next_target_id(), "T000002");
    }

    #[test]
    fn test_target_id_generation_per_radar() {
        let mut tracker = TargetTracker::new_per_radar(1, 2048);
        assert_eq!(tracker.next_target_id(), "T100001");
        assert_eq!(tracker.next_target_id(), "T100002");
    }

    #[test]
    fn test_target_id_wrap() {
        let mut tracker = TargetTracker::new_merged(2048);
        tracker.next_id = 999999;
        assert_eq!(tracker.next_target_id(), "T999999");
        assert_eq!(tracker.next_target_id(), "T000000");
        assert_eq!(tracker.next_target_id(), "T000001");
    }

    #[test]
    fn test_calculate_distance() {
        let p1 = GeoPosition::new(52.0, 4.0);
        let p2 = GeoPosition::new(52.001, 4.0); // ~111m north

        let dist = calculate_distance(&p1, &p2);
        assert!(dist > 100.0 && dist < 120.0, "Distance was {}", dist);
    }

    #[test]
    fn test_calculate_bearing_north() {
        let p1 = GeoPosition::new(52.0, 4.0);
        let p2 = GeoPosition::new(52.001, 4.0); // North

        let bearing = calculate_bearing(&p1, &p2);
        assert!(bearing.abs() < 0.01, "Bearing north was {}", bearing);
    }

    #[test]
    fn test_calculate_bearing_east() {
        let p1 = GeoPosition::new(52.0, 4.0);
        let p2 = GeoPosition::new(52.0, 4.001); // East

        let bearing = calculate_bearing(&p1, &p2);
        let expected = PI / 2.0; // 90 degrees
        assert!(
            (bearing - expected).abs() < 0.01,
            "Bearing east was {} (expected {})",
            bearing.to_degrees(),
            expected.to_degrees()
        );
    }

    #[test]
    fn test_new_acquiring_target() {
        let mut tracker = TargetTracker::new_merged(2048);

        let candidate = make_candidate(52.0, 4.0, 1000);
        let result = tracker.process_candidate(candidate);

        // First candidate creates an active target in Acquiring status
        assert!(matches!(result, ProcessResult::NewAcquiring(_)));
        assert_eq!(tracker.active_count(), 1);

        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Acquiring);
    }

    #[test]
    fn test_promote_to_tracking() {
        let mut tracker = TargetTracker::new_merged(2048);

        // First candidate - creates acquiring target
        let candidate1 = make_candidate(52.0, 4.0, 1000);
        tracker.process_candidate(candidate1);
        assert_eq!(tracker.active_count(), 1);

        // Second candidate nearby - should promote to tracking and establish COG
        let candidate2 = make_candidate(52.0001, 4.0, 4000); // 3s later, ~11m north
        let result = tracker.process_candidate(candidate2);

        assert!(matches!(result, ProcessResult::Promoted(_)));
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Tracking);
        assert!(target.cog.is_some());
    }

    #[test]
    fn test_active_target_matching() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Create an active target via promotion
        let candidate1 = make_candidate(52.0, 4.0, 1000);
        tracker.process_candidate(candidate1);

        let candidate2 = make_candidate(52.0001, 4.0, 4000);
        tracker.process_candidate(candidate2);

        assert_eq!(tracker.active_count(), 1);

        // Third candidate should match the active target
        let candidate3 = make_candidate(52.0002, 4.0, 7000);
        tracker.process_candidate(candidate3);

        // Still just one active target (it was updated, not duplicated)
        assert_eq!(tracker.active_count(), 1);
    }

    #[test]
    fn test_acquiring_timeout() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Add a candidate - now creates an active target in Acquiring status
        let candidate = make_candidate(52.0, 4.0, 1000);
        tracker.process_candidate(candidate);
        assert_eq!(tracker.active_count(), 1);

        // Check at 10 seconds - acquiring targets should become lost
        let (deleted, lost) = tracker.check_timeouts(11_000);
        assert!(deleted.is_empty());
        assert_eq!(lost.len(), 1);

        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Lost);
    }

    #[test]
    fn test_revolution_detection() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Simulate spokes without wrap
        tracker.check_revolution(100, 1000);
        tracker.check_revolution(200, 1000);
        tracker.check_revolution(300, 1000);
        assert_eq!(tracker.revolution_count, 0);

        // Wrap around (high to low = revolution complete)
        tracker.check_revolution(2000, 1000);
        tracker.check_revolution(100, 2000);
        assert_eq!(tracker.revolution_count, 1);
    }

    #[test]
    fn test_no_match_too_far_apart() {
        let mut tracker = TargetTracker::new_merged(2048);

        // First candidate - creates active target
        let candidate1 = make_candidate(52.0, 4.0, 1000);
        tracker.process_candidate(candidate1);

        // Second candidate very far away - should not match, creates new target
        let candidate2 = make_candidate(53.0, 5.0, 4000); // ~100km away
        tracker.process_candidate(candidate2);

        // Should have two separate active targets (both in Acquiring status)
        assert_eq!(tracker.active_count(), 2);
    }

    #[test]
    fn test_active_target_update() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Create active target via promotion
        tracker.process_candidate(make_candidate(52.0, 4.0, 0));
        tracker.process_candidate(make_candidate(52.0001, 4.0, 3000));

        assert_eq!(tracker.active_count(), 1);

        // Update with moving target
        tracker.process_candidate(make_candidate(52.0002, 4.0, 6000));
        tracker.process_candidate(make_candidate(52.0003, 4.0, 9000));

        // Get the active target and check SOG
        let target = tracker.get_active_targets().next().unwrap();
        assert!(target.sog.is_some(), "SOG should be set");
        assert!(target.sog.unwrap() > 0.0, "SOG should be positive: {:?}", target.sog);
        assert!(target.cog.is_some(), "COG should be set");
        assert!(target.update_count >= 3, "Update count: {}", target.update_count);
    }

    #[test]
    fn test_target_lost_timeout() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Create an active target at time 0
        let candidate1 = make_candidate(52.0, 4.0, 0);
        tracker.process_candidate(candidate1);
        // Second candidate at 3s - last_update becomes 3000
        let candidate2 = make_candidate(52.0001, 4.0, 3000);
        tracker.process_candidate(candidate2);

        assert_eq!(tracker.active_count(), 1);

        // Check at 22 seconds (19s after last_update=3000) - should still be Tracking
        let (deleted, lost) = tracker.check_timeouts(22_000);
        assert!(deleted.is_empty());
        assert!(lost.is_empty());
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Tracking);

        // Check at 24 seconds (21s after last_update=3000) - should be Lost
        let (deleted, lost) = tracker.check_timeouts(24_000);
        assert!(deleted.is_empty());
        assert_eq!(lost.len(), 1);
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Lost);

        // Check again - should not re-report as lost
        let (deleted, lost) = tracker.check_timeouts(28_000);
        assert!(deleted.is_empty());
        assert!(lost.is_empty());
    }

    #[test]
    fn test_target_deleted_timeout() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Create an active target at time 0
        let candidate1 = make_candidate(52.0, 4.0, 0);
        tracker.process_candidate(candidate1);
        // Second candidate at 3s - last_update becomes 3000
        let candidate2 = make_candidate(52.0001, 4.0, 3000);
        tracker.process_candidate(candidate2);

        assert_eq!(tracker.active_count(), 1);

        // Check at 34 seconds (31s after last_update=3000) - should be deleted
        let (deleted, _lost) = tracker.check_timeouts(34_000);
        assert_eq!(deleted.len(), 1);
        assert_eq!(tracker.active_count(), 0);
    }

    #[test]
    fn test_target_recovers_from_lost() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Create an active target using MARPA (larger uncertainty)
        let candidate = make_candidate(52.0, 4.0, 0);
        tracker.add_active_target(&candidate);

        // Update at 3s to establish velocity
        let candidate2 = make_candidate(52.0001, 4.0, 3000);
        tracker.process_candidate(candidate2);

        // Get the target's predicted position at 29000ms for reference
        let target = tracker.get_active_targets().next().unwrap();
        let predicted = target.predict_position(29_000);

        // Mark as lost (28s = 25s after last_update at 3000)
        let (_, lost) = tracker.check_timeouts(28_000);
        assert_eq!(lost.len(), 1);
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Lost);

        // Target is seen again at the predicted position - should recover
        let candidate3 = make_candidate(predicted.lat(), predicted.lon(), 29_000);
        tracker.process_candidate(candidate3);

        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Tracking);
    }

    #[test]
    fn test_stationary_target_extended_timeout() {
        let mut tracker = TargetTracker::new_merged(2048);

        // Create a stationary target (buoy) - same position for multiple updates
        // Need MIN_UPDATES_FOR_STATIONARY (5) updates at same position
        let pos = (52.0, 4.0);
        for i in 0..6 {
            let candidate = make_candidate(pos.0, pos.1, i * 3000);
            tracker.process_candidate(candidate);
        }

        assert_eq!(tracker.active_count(), 1);
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Tracking);
        assert!(target.sog.unwrap() < 0.5, "SOG should be near zero: {:?}", target.sog);
        assert!(target.update_count >= 5);

        // Last update was at 15000ms
        // At 35000ms (20s after last update) - normal target would be lost
        // But stationary target has 60s timeout, so should still be tracking
        let (deleted, lost) = tracker.check_timeouts(35_000);
        assert!(deleted.is_empty());
        assert!(lost.is_empty());
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Tracking);

        // At 76000ms (61s after last update at 15000) - should be lost
        let (deleted, lost) = tracker.check_timeouts(76_000);
        assert!(deleted.is_empty());
        assert_eq!(lost.len(), 1);
        let target = tracker.get_active_targets().next().unwrap();
        assert_eq!(target.status, TargetStatus::Lost);

        // At 136000ms (121s after last update) - should be deleted
        let (deleted, _lost) = tracker.check_timeouts(136_000);
        assert_eq!(deleted.len(), 1);
        assert_eq!(tracker.active_count(), 0);
    }

    #[test]
    fn test_circling_target_tracks_continuously() {
        // Simulates a boat circling at 15 knots in a 250m radius circle
        // With forced position override, the tracker should maintain track
        // through continuous turns by blending measured velocities.

        let mut tracker = TargetTracker::new_merged(2048);

        // Circle parameters (matching emulator world.rs)
        let radius_m = 250.0;
        let speed_knots = 15.0;
        let speed_ms = speed_knots * 1852.0 / 3600.0; // ~7.72 m/s
        let angular_velocity = speed_ms / radius_m; // ~0.031 rad/s

        // Center of circle is 350m north of radar (at 52.0, 4.0)
        let radar_lat = 52.0;
        let radar_lon = 4.0;
        let center_lat = radar_lat + 350.0 / METERS_PER_DEGREE_LATITUDE;
        let center_lon = radar_lon;

        // Helper to calculate position on circle at given angle
        // angle=0 is south of center (closest to radar), increasing clockwise
        let position_at_angle = |angle: f64| -> (f64, f64) {
            // bearing from center: south + angle
            let bearing = PI + angle;
            let lat = center_lat + radius_m * bearing.cos() / METERS_PER_DEGREE_LATITUDE;
            let lon = center_lon
                + radius_m * bearing.sin() / meters_per_degree_longitude(&center_lat);
            (lat, lon)
        };

        // Revolution time ~3 seconds (typical radar)
        let revolution_ms = 3000u64;

        // Start tracking: first detection at angle=0 (south of center)
        let (lat0, lon0) = position_at_angle(0.0);
        let candidate0 = make_candidate(lat0, lon0, 0);
        tracker.process_candidate(candidate0);

        // Second detection after one revolution - angle increases by angular_velocity * 3s
        let angle1 = angular_velocity * 3.0;
        let (lat1, lon1) = position_at_angle(angle1);
        let candidate1 = make_candidate(lat1, lon1, revolution_ms);
        let result1 = tracker.process_candidate(candidate1);
        assert!(
            matches!(result1, ProcessResult::Promoted(_)),
            "Should promote to tracking: {:?}",
            result1
        );

        // Continue tracking through multiple full circles (180 seconds = 60 revolutions)
        // This covers ~5.5 full circles at 32.5s per circle
        let mut successful_updates = 0;
        let mut lost_count = 0;
        let target_id = match result1 {
            ProcessResult::Promoted(id) => id,
            _ => panic!("Expected Promoted"),
        };

        for rev in 2..60 {
            let time = rev * revolution_ms;
            let angle = angular_velocity * (time as f64 / 1000.0);
            let (lat, lon) = position_at_angle(angle);

            let candidate = make_candidate(lat, lon, time);
            let result = tracker.process_candidate(candidate);

            match result {
                ProcessResult::Updated(id) if id == target_id => {
                    successful_updates += 1;
                }
                ProcessResult::NewAcquiring(_) => {
                    lost_count += 1;
                }
                _ => {}
            }
        }

        // With forced position override for fast targets, we should maintain tracking
        // through the entire test (58 updates after initial 2)
        let total_circles = (angular_velocity * 60.0 * 3.0) / (2.0 * PI);
        println!(
            "Circling target: {} successful updates, {} lost, {:.1} full circles completed",
            successful_updates, lost_count, total_circles
        );

        // Should have maintained tracking for at least 90% of updates
        assert!(
            successful_updates >= 50,
            "Expected at least 50 successful updates for circling target, got {}",
            successful_updates
        );

        // Should have only one active target (no fragmentation)
        assert_eq!(
            tracker.active_count(),
            1,
            "Expected exactly 1 target for circling boat, got {}",
            tracker.active_count()
        );

        // Verify target has reasonable speed estimate (should be close to 15 knots = 7.72 m/s)
        let target = tracker.get_target(&target_id).unwrap();
        let tracked_speed = target.sog.unwrap_or(0.0);
        assert!(
            (tracked_speed - speed_ms).abs() < 2.0,
            "Target speed {:.1} m/s should be close to actual {:.1} m/s",
            tracked_speed,
            speed_ms
        );
    }
}
