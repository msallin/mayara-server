//! ARPA (Automatic Radar Plotting Aid) - automatic target detection via guard zones
//!
//! This module handles automatic target acquisition within configured guard zones.
//! Targets detected here are passed to the main target tracking system.
//!
//! ## Coordinate Systems
//!
//! This module uses two coordinate systems for angles:
//! - `SpokeAngle`: Angle relative to ship's bow (0 = ahead). Used for guard zone configuration.
//! - `SpokeBearing`: Bearing relative to True North (0 = North). Used for blob tracking.
//! - `SpokeHeading`: Ship's heading (bearing of bow relative to North). Used for conversion.
//!
//! Relationship: `bearing = angle + heading` (mod spokes)

use std::cmp::{max, min};

use super::spoke_coords::{SpokeAngle, SpokeBearing, SpokeHeading};
use super::{MAX_BLOB_PIXELS, MIN_BLOB_PIXELS, MIN_BLOB_RANGE};
use crate::radar::GeoPosition;

/// Guard zone configuration for automatic target detection.
/// Angles are stored relative to ship's bow (SpokeAngle), distances in pixels.
#[derive(Debug, Clone)]
pub(crate) struct DetectionGuardZone {
    /// Start angle relative to ship's bow (0 = ahead)
    pub(crate) start_angle: SpokeAngle,
    /// End angle relative to ship's bow (0 = ahead)
    pub(crate) end_angle: SpokeAngle,
    /// Inner distance in pixels
    pub(crate) inner_range: i32,
    /// Outer distance in pixels
    pub(crate) outer_range: i32,
    /// Whether this guard zone is enabled for detection
    pub(crate) enabled: bool,
    /// Last scan time for each bearing to avoid duplicate detections
    last_scan_time: Vec<u64>,
    // Original config values (for recalculation when pixels_per_meter changes)
    config_start_angle_rad: f64,
    config_end_angle_rad: f64,
    /// Inner range in meters (from config)
    pub(crate) config_inner_range_m: f64,
    /// Outer range in meters (from config)
    pub(crate) config_outer_range_m: f64,
    config_enabled: bool,
}

impl DetectionGuardZone {
    pub(crate) fn new(spokes_per_revolution: i32) -> Self {
        Self {
            start_angle: SpokeAngle::from_raw(0),
            end_angle: SpokeAngle::from_raw(0),
            inner_range: 0,
            outer_range: 0,
            enabled: false,
            last_scan_time: vec![0; spokes_per_revolution as usize],
            config_start_angle_rad: 0.0,
            config_end_angle_rad: 0.0,
            config_inner_range_m: 0.0,
            config_outer_range_m: 0.0,
            config_enabled: false,
        }
    }

    /// Update zone from config (angles in radians, distances in meters)
    pub(crate) fn update_from_config(
        &mut self,
        start_angle_rad: f64,
        end_angle_rad: f64,
        inner_range_m: f64,
        outer_range_m: f64,
        enabled: bool,
        spokes_per_revolution: i32,
        pixels_per_meter: f64,
    ) {
        // Store original config values for later recalculation
        self.config_start_angle_rad = start_angle_rad;
        self.config_end_angle_rad = end_angle_rad;
        self.config_inner_range_m = inner_range_m;
        self.config_outer_range_m = outer_range_m;
        self.config_enabled = enabled;

        self.recalculate(spokes_per_revolution, pixels_per_meter);
    }

    /// Recalculate pixel/spoke values from stored config when pixels_per_meter changes
    pub(crate) fn recalculate(&mut self, spokes_per_revolution: i32, pixels_per_meter: f64) {
        self.enabled = self.config_enabled && pixels_per_meter > 0.0;

        if !self.enabled {
            return;
        }

        let spokes = spokes_per_revolution as u32;

        // Convert angles from radians to SpokeAngle (relative to bow)
        self.start_angle = SpokeAngle::from_radians(self.config_start_angle_rad, spokes);
        self.end_angle = SpokeAngle::from_radians(self.config_end_angle_rad, spokes);

        // Convert distances from meters to pixels
        self.inner_range = (self.config_inner_range_m * pixels_per_meter).max(1.0) as i32;
        self.outer_range = (self.config_outer_range_m * pixels_per_meter) as i32;

        // Calculate implied radar range (assuming 1024 pixels per spoke)
        let implied_range_m = if pixels_per_meter > 0.0 {
            1024.0 / pixels_per_meter
        } else {
            0.0
        };

        log::info!(
            "GuardZone configured: angles={}..{} spokes (relative to bow), range={}..{} pixels (config {}..{}m, ppm={:.4}, radar_range={:.0}m)",
            self.start_angle,
            self.end_angle,
            self.inner_range,
            self.outer_range,
            self.config_inner_range_m,
            self.config_outer_range_m,
            pixels_per_meter,
            implied_range_m
        );
    }

    /// Check if this zone has config that needs recalculation
    pub(crate) fn has_pending_config(&self) -> bool {
        self.config_enabled && !self.enabled
    }

    /// Check if an angle (relative to bow) is within this guard zone
    fn contains_angle(&self, angle: SpokeAngle, spokes: u32) -> bool {
        if !self.enabled {
            return false;
        }
        angle.is_between(self.start_angle, self.end_angle, spokes)
    }

    /// Check if a range (in pixels) is within this guard zone
    fn contains_range(&self, range: i32) -> bool {
        let in_range = self.enabled && range >= self.inner_range && range <= self.outer_range;
        // Log when a range check fails near the outer boundary
        if self.enabled && !in_range && range > self.outer_range && range <= self.outer_range + 50 {
            log::trace!(
                "Range {} just outside outer boundary {} (inner={})",
                range,
                self.outer_range,
                self.inner_range
            );
        }
        in_range
    }

    /// Check if a position is within the guard zone
    /// bearing: the bearing in geographic coordinates (0 = North)
    /// heading: the ship's heading (to convert geographic bearing to relative angle)
    pub(crate) fn contains(
        &self,
        bearing: SpokeBearing,
        range: i32,
        spokes: u32,
        heading: SpokeHeading,
    ) -> bool {
        // Convert geographic bearing to relative angle by subtracting heading
        // Guard zone angles are stored relative to ship heading
        let relative_angle = bearing.to_angle(heading, spokes);
        self.contains_angle(relative_angle, spokes) && self.contains_range(range)
    }
}

/// A blob being incrementally built as spokes arrive.
/// Used for automatic target detection within guard zones.
/// Bearings are geographic (relative to True North).
#[derive(Debug, Clone)]
pub(crate) struct BlobInProgress {
    /// Range values present on the last spoke that contributed to this blob
    /// Used to check adjacency with the next spoke
    last_spoke_ranges: Vec<i32>,
    /// The bearing of the last spoke that contributed pixels (geographic)
    last_bearing: SpokeBearing,
    /// Bounding box (geographic bearings)
    pub(crate) min_bearing: SpokeBearing,
    pub(crate) max_bearing: SpokeBearing,
    pub(crate) min_r: i32,
    pub(crate) max_r: i32,
    /// Total pixel count
    pub(crate) pixel_count: usize,
    /// Time when first pixel was seen
    pub(crate) start_time: u64,
    /// Own ship position when blob started
    pub(crate) start_pos: GeoPosition,
    /// Heading when blob was first detected (for guard zone validation)
    pub(crate) start_heading: SpokeHeading,
}

impl BlobInProgress {
    pub(crate) fn new(bearing: SpokeBearing, r: i32, time: u64, pos: GeoPosition, heading: SpokeHeading) -> Self {
        Self {
            last_spoke_ranges: vec![r],
            last_bearing: bearing,
            min_bearing: bearing,
            max_bearing: bearing,
            min_r: r,
            max_r: r,
            pixel_count: 1,
            start_time: time,
            start_pos: pos,
            start_heading: heading,
        }
    }

    /// Add a pixel to this blob
    pub(crate) fn add_pixel(&mut self, bearing: SpokeBearing, r: i32) {
        self.max_bearing = bearing; // bearing always increases as we process spokes
        self.min_r = min(self.min_r, r);
        self.max_r = max(self.max_r, r);
        self.pixel_count += 1;
    }

    /// Start a new spoke - clear last_spoke_ranges and set last_bearing
    pub(crate) fn start_new_spoke(&mut self, bearing: SpokeBearing) {
        self.last_spoke_ranges.clear();
        self.last_bearing = bearing;
    }

    /// Check if a range value on the current spoke is adjacent to this blob
    /// (i.e., within 1 pixel of any range on the previous spoke)
    pub(crate) fn is_adjacent(&self, r: i32) -> bool {
        self.last_spoke_ranges
            .iter()
            .any(|&prev_r| (prev_r - r).abs() <= 1)
    }

    /// Get the last bearing that contributed to this blob
    pub(crate) fn last_bearing(&self) -> SpokeBearing {
        self.last_bearing
    }

    /// Add a range to the last spoke ranges
    pub(crate) fn push_last_spoke_range(&mut self, r: i32) {
        self.last_spoke_ranges.push(r);
    }

    /// Calculate center position in polar coordinates (raw bearing, r)
    pub(crate) fn center(&self, spokes: u32) -> (i32, i32) {
        // Calculate center bearing - handle wraparound
        let min_b = self.min_bearing.as_i32();
        let max_b = self.max_bearing.as_i32();
        let center_bearing = if max_b >= min_b {
            (min_b + max_b) / 2
        } else {
            // Wraparound case
            let sum = min_b + max_b + spokes as i32;
            (sum / 2) % spokes as i32
        };
        let center_r = (self.min_r + self.max_r) / 2;
        (center_bearing, center_r)
    }

    /// Check if blob meets minimum size requirements
    pub(crate) fn is_valid(&self) -> bool {
        self.pixel_count >= MIN_BLOB_PIXELS
            && self.pixel_count <= MAX_BLOB_PIXELS
            && self.min_r >= MIN_BLOB_RANGE
    }
}

/// ARPA detector - handles automatic target detection within guard zones
#[derive(Debug, Clone)]
pub(crate) struct ArpaDetector {
    /// Guard zones for target detection
    pub(crate) guard_zones: [DetectionGuardZone; 2],
    /// Current heading (updated each spoke, used for guard zone checks)
    pub(crate) current_heading: SpokeHeading,
    /// Blobs currently being built as spokes arrive
    blobs_in_progress: Vec<BlobInProgress>,
    /// Previous bearing for detecting spoke gaps (geographic)
    prev_bearing: SpokeBearing,
}

impl ArpaDetector {
    pub(crate) fn new(spokes_per_revolution: i32) -> Self {
        Self {
            guard_zones: [
                DetectionGuardZone::new(spokes_per_revolution),
                DetectionGuardZone::new(spokes_per_revolution),
            ],
            current_heading: SpokeHeading::zero(),
            blobs_in_progress: Vec::new(),
            prev_bearing: SpokeBearing::from_raw(0),
        }
    }

    /// Check if any guard zone is enabled
    pub(crate) fn has_active_guard_zone(&self) -> bool {
        self.guard_zones.iter().any(|z| z.enabled)
    }

    /// Check if a position is within any enabled guard zone
    pub(crate) fn is_in_guard_zone(
        &self,
        bearing: SpokeBearing,
        range: i32,
        heading: SpokeHeading,
        spokes: u32,
    ) -> bool {
        self.guard_zones
            .iter()
            .any(|z| z.contains(bearing, range, spokes, heading))
    }

    /// Get which guard zone (1 or 2) contains the position, or 0 if none
    pub(crate) fn get_containing_zone(
        &self,
        bearing: SpokeBearing,
        range: i32,
        heading: SpokeHeading,
        spokes: u32,
    ) -> u8 {
        if self.guard_zones[0].contains(bearing, range, spokes, heading) {
            1
        } else if self.guard_zones[1].contains(bearing, range, spokes, heading) {
            2
        } else {
            0
        }
    }

    /// Recalculate guard zones when pixels_per_meter changes
    pub(crate) fn recalculate_zones(&mut self, spokes_per_revolution: i32, pixels_per_meter: f64) {
        for zone in &mut self.guard_zones {
            if zone.has_pending_config() || zone.enabled {
                zone.recalculate(spokes_per_revolution, pixels_per_meter);
            }
        }
    }

    /// Process strong pixels from a spoke and update blobs in progress.
    /// Returns completed blobs that should be passed to target acquisition.
    ///
    /// Parameters:
    /// - `bearing`: Geographic bearing of the spoke (0 = North)
    /// - `heading`: Ship's heading (bearing of bow)
    /// - `strong_pixels`: Range values of strong pixels on this spoke
    /// - `time`: Time of spoke
    /// - `pos`: Ship position at time of spoke
    /// - `spokes`: Number of spokes per revolution
    pub(crate) fn process_blob_pixels(
        &mut self,
        bearing: SpokeBearing,
        heading: SpokeHeading,
        strong_pixels: &[i32],
        time: u64,
        pos: GeoPosition,
        spokes: u32,
    ) -> Vec<BlobInProgress> {
        let mut completed_blobs = Vec::new();

        // Check if any guard zone is enabled - automatic target acquisition ONLY happens
        // within enabled guard zones (matching C++ radar_pi behavior)
        let guard_zone_active = self.guard_zones[0].enabled || self.guard_zones[1].enabled;

        // Log guard zone state once per rotation (at bearing 0)
        if bearing.raw() == 0 {
            log::debug!(
                "Guard zone state: active={}, zone0_enabled={}, zone0 angles {}..{} range {}..{}, zone1_enabled={}",
                guard_zone_active,
                self.guard_zones[0].enabled,
                self.guard_zones[0].start_angle,
                self.guard_zones[0].end_angle,
                self.guard_zones[0].inner_range,
                self.guard_zones[0].outer_range,
                self.guard_zones[1].enabled
            );
        }

        // No automatic target acquisition without guard zones
        if !guard_zone_active {
            // Complete any in-progress blobs before returning
            if !self.blobs_in_progress.is_empty() {
                completed_blobs = self.blobs_in_progress.drain(..).collect();
            }
            self.prev_bearing = bearing;
            return completed_blobs;
        }

        // Handle bearing wraparound - complete all blobs when we wrap
        if !self.blobs_in_progress.is_empty() {
            let first_blob_bearing = self.blobs_in_progress[0].min_bearing;
            // If we've wrapped around and are back near where blobs started, complete them all
            if bearing.raw() < first_blob_bearing.raw() && self.prev_bearing.raw() > bearing.raw() {
                completed_blobs.extend(self.blobs_in_progress.drain(..));
            }
        }

        // Filter pixels by guard zone and group into contiguous runs
        // A run is a sequence of pixels where each is adjacent (r differs by 1)
        let mut runs: Vec<Vec<i32>> = Vec::new();
        let mut current_run: Vec<i32> = Vec::new();

        // Log guard zone range checking periodically (every 256 bearings, on first strong pixel)
        if bearing.raw() % 256 == 0 && !strong_pixels.is_empty() {
            let first_r = strong_pixels[0];
            let last_r = *strong_pixels.last().unwrap();
            log::debug!(
                "GZ range check: spoke pixels {}..{}, gz0 range={}..{} ({}), gz1 range={}..{} ({})",
                first_r,
                last_r,
                self.guard_zones[0].inner_range,
                self.guard_zones[0].outer_range,
                if self.guard_zones[0].enabled { "on" } else { "off" },
                self.guard_zones[1].inner_range,
                self.guard_zones[1].outer_range,
                if self.guard_zones[1].enabled { "on" } else { "off" }
            );
        }

        for &r in strong_pixels {
            // Check if pixel is in a guard zone
            let in_zone = self.guard_zones[0].contains(bearing, r, spokes, heading)
                || self.guard_zones[1].contains(bearing, r, spokes, heading);
            if !in_zone {
                // End current run if any
                if !current_run.is_empty() {
                    runs.push(std::mem::take(&mut current_run));
                }
                continue;
            }

            // Check if this pixel is adjacent to the last pixel in current run
            if current_run.is_empty() || r == current_run.last().unwrap() + 1 {
                current_run.push(r);
            } else {
                // Start a new run
                runs.push(std::mem::take(&mut current_run));
                current_run.push(r);
            }
        }
        // Don't forget the last run
        if !current_run.is_empty() {
            runs.push(current_run);
        }

        let mut run_assigned: Vec<bool> = vec![false; runs.len()];
        let prev_bearing = bearing.sub(1, spokes);

        // For each run, find ALL adjacent blobs (there may be multiple that need merging)
        for (run_idx, run) in runs.iter().enumerate() {
            let mut adjacent_blob_indices: Vec<usize> = Vec::new();

            for (blob_idx, blob) in self.blobs_in_progress.iter().enumerate() {
                // Only consider blobs whose last_bearing is the previous spoke
                if blob.last_bearing() != prev_bearing {
                    continue;
                }
                // Check if any pixel in the run is adjacent to the blob
                for &r in run {
                    if blob.is_adjacent(r) {
                        adjacent_blob_indices.push(blob_idx);
                        break;
                    }
                }
            }

            if adjacent_blob_indices.is_empty() {
                continue;
            }

            run_assigned[run_idx] = true;

            // If multiple blobs are adjacent to this run, merge them all into the first one
            let primary_idx = adjacent_blob_indices[0];

            // First, merge any additional blobs into the primary blob
            // Process in reverse order to preserve indices during removal
            for &merge_idx in adjacent_blob_indices.iter().skip(1).rev() {
                let merge_blob = self.blobs_in_progress.remove(merge_idx);
                let primary = &mut self.blobs_in_progress[if merge_idx < primary_idx {
                    primary_idx - 1
                } else {
                    primary_idx
                }];
                // Merge the blob data - use raw values for min comparison since SpokeBearing
                // doesn't have a natural ordering (it wraps around)
                if merge_blob.min_bearing.raw() < primary.min_bearing.raw() {
                    primary.min_bearing = merge_blob.min_bearing;
                }
                primary.min_r = min(primary.min_r, merge_blob.min_r);
                primary.max_r = max(primary.max_r, merge_blob.max_r);
                primary.pixel_count += merge_blob.pixel_count;
                // Note: last_spoke_ranges from merged blob are from prev spoke, not needed
            }

            // Now extend the primary blob with this run
            // Need to recalculate primary_idx as it may have changed due to removals
            let adjusted_primary_idx = adjacent_blob_indices[0]
                - adjacent_blob_indices
                    .iter()
                    .skip(1)
                    .filter(|&&i| i < adjacent_blob_indices[0])
                    .count();
            let blob = &mut self.blobs_in_progress[adjusted_primary_idx];
            blob.start_new_spoke(bearing);
            for &r in run {
                blob.add_pixel(bearing, r);
                blob.push_last_spoke_range(r);
            }
        }

        // Start new blobs for unassigned runs
        for (run_idx, run) in runs.iter().enumerate() {
            if run_assigned[run_idx] {
                continue;
            }
            // Create a new blob with all pixels in this run
            // Store heading so we can validate guard zone membership when blob completes
            let mut blob = BlobInProgress::new(bearing, run[0], time, pos.clone(), heading);
            for &r in run.iter().skip(1) {
                blob.add_pixel(bearing, r);
                blob.push_last_spoke_range(r);
            }
            self.blobs_in_progress.push(blob);
        }

        // Find completed blobs: those that weren't extended this spoke
        // AND whose last_bearing is before the previous spoke (so they had a gap)
        let mut completed_indices: Vec<usize> = Vec::new();
        for (idx, blob) in self.blobs_in_progress.iter().enumerate() {
            // Blob is complete if it wasn't extended and last_bearing < prev_bearing
            // (meaning there's been at least one spoke with no contribution)
            if blob.last_bearing() != bearing && blob.last_bearing() != prev_bearing {
                completed_indices.push(idx);
            }
        }

        // Process completed blobs (in reverse order to preserve indices during removal)
        for &idx in completed_indices.iter().rev() {
            let blob = self.blobs_in_progress.remove(idx);
            completed_blobs.push(blob);
        }

        self.prev_bearing = bearing;
        completed_blobs
    }

    /// Complete all blobs in progress (called on angle wraparound or range change)
    pub(crate) fn complete_all_blobs(&mut self) -> Vec<BlobInProgress> {
        self.blobs_in_progress.drain(..).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPOKES: u32 = 2048;
    const SPOKES_I32: i32 = 2048;

    /// Helper to create a guard zone with angles in degrees and ranges in pixels
    fn make_guard_zone(
        start_angle_deg: f64,
        end_angle_deg: f64,
        inner_range_px: i32,
        outer_range_px: i32,
    ) -> DetectionGuardZone {
        let mut zone = DetectionGuardZone::new(SPOKES_I32);
        // Set config values directly in radians (bypassing meters conversion)
        zone.config_start_angle_rad = start_angle_deg.to_radians();
        zone.config_end_angle_rad = end_angle_deg.to_radians();
        zone.config_enabled = true;
        // For testing, set pixel values directly
        zone.start_angle = SpokeAngle::from_radians(start_angle_deg.to_radians(), SPOKES);
        zone.end_angle = SpokeAngle::from_radians(end_angle_deg.to_radians(), SPOKES);
        zone.inner_range = inner_range_px;
        zone.outer_range = outer_range_px;
        zone.enabled = true;
        zone
    }

    /// Helper to create an ArpaDetector with a configured guard zone
    fn make_detector_with_zone(
        start_angle_deg: f64,
        end_angle_deg: f64,
        inner_range_px: i32,
        outer_range_px: i32,
    ) -> ArpaDetector {
        let mut detector = ArpaDetector::new(SPOKES_I32);
        detector.guard_zones[0] = make_guard_zone(
            start_angle_deg,
            end_angle_deg,
            inner_range_px,
            outer_range_px,
        );
        detector
    }

    /// Simulate spokes with a target at a specific bearing and range
    /// Returns completed blobs after processing all spokes
    fn simulate_target(
        detector: &mut ArpaDetector,
        target_bearing_start: u32,
        target_bearing_end: u32,
        target_range: i32,
        heading: SpokeHeading,
    ) -> Vec<BlobInProgress> {
        let pos = GeoPosition {
            lat: 52.0,
            lon: 4.0,
        };
        let time = 1000u64;

        let mut all_completed = Vec::new();

        // Process spokes around the target area
        // Start a few spokes before the target, go through it, and a few after
        let start = if target_bearing_start > 5 {
            target_bearing_start - 5
        } else {
            0
        };
        let end = (target_bearing_end + 10).min(SPOKES - 1);

        for spoke in start..=end {
            let bearing = SpokeBearing::from_raw(spoke);

            // Generate strong pixels only for spokes within the target bearing range
            let strong_pixels = if spoke >= target_bearing_start && spoke <= target_bearing_end {
                // Simulate a target that spans a few pixels in range
                vec![target_range - 2, target_range - 1, target_range, target_range + 1, target_range + 2]
            } else {
                vec![]
            };

            let completed = detector.process_blob_pixels(
                bearing,
                heading,
                &strong_pixels,
                time + spoke as u64,
                pos.clone(),
                SPOKES,
            );
            all_completed.extend(completed);
        }

        all_completed
    }

    // =========================================================================
    // Guard Zone Range Tests
    // =========================================================================

    #[test]
    fn test_guard_zone_contains_range_inside() {
        let zone = make_guard_zone(0.0, 90.0, 100, 500);

        // Range inside zone
        assert!(zone.contains_range(100), "inner boundary should be inside");
        assert!(zone.contains_range(300), "middle should be inside");
        assert!(zone.contains_range(500), "outer boundary should be inside");
    }

    #[test]
    fn test_guard_zone_contains_range_outside() {
        let zone = make_guard_zone(0.0, 90.0, 100, 500);

        // Range outside zone
        assert!(!zone.contains_range(50), "before inner should be outside");
        assert!(!zone.contains_range(99), "just before inner should be outside");
        assert!(!zone.contains_range(501), "just after outer should be outside");
        assert!(!zone.contains_range(1000), "far after outer should be outside");
    }

    #[test]
    fn test_guard_zone_disabled_rejects_all() {
        let mut zone = make_guard_zone(0.0, 90.0, 100, 500);
        zone.enabled = false;

        assert!(!zone.contains_range(300), "disabled zone should reject all ranges");
    }

    // =========================================================================
    // Guard Zone Angle Tests (relative to bow)
    // =========================================================================

    #[test]
    fn test_guard_zone_contains_angle_simple() {
        // Zone from 0° to 90° (straight ahead to starboard)
        let zone = make_guard_zone(0.0, 90.0, 100, 500);

        // Test angles relative to bow
        let angle_0 = SpokeAngle::from_radians(0.0, SPOKES);
        let angle_45 = SpokeAngle::from_radians(45.0_f64.to_radians(), SPOKES);
        let angle_90 = SpokeAngle::from_radians(90.0_f64.to_radians(), SPOKES);
        let angle_180 = SpokeAngle::from_radians(180.0_f64.to_radians(), SPOKES);
        let angle_270 = SpokeAngle::from_radians(270.0_f64.to_radians(), SPOKES);

        assert!(zone.contains_angle(angle_0, SPOKES), "0° should be inside 0-90°");
        assert!(zone.contains_angle(angle_45, SPOKES), "45° should be inside 0-90°");
        assert!(zone.contains_angle(angle_90, SPOKES), "90° should be inside 0-90°");
        assert!(!zone.contains_angle(angle_180, SPOKES), "180° should be outside 0-90°");
        assert!(!zone.contains_angle(angle_270, SPOKES), "270° should be outside 0-90°");
    }

    #[test]
    fn test_guard_zone_contains_angle_wraparound() {
        // Zone from 270° to 90° (wraps around 0°, covers port quarter through bow to starboard)
        let zone = make_guard_zone(270.0, 90.0, 100, 500);

        let angle_0 = SpokeAngle::from_radians(0.0, SPOKES);
        let angle_45 = SpokeAngle::from_radians(45.0_f64.to_radians(), SPOKES);
        let angle_90 = SpokeAngle::from_radians(90.0_f64.to_radians(), SPOKES);
        let angle_180 = SpokeAngle::from_radians(180.0_f64.to_radians(), SPOKES);
        let angle_270 = SpokeAngle::from_radians(270.0_f64.to_radians(), SPOKES);
        let angle_315 = SpokeAngle::from_radians(315.0_f64.to_radians(), SPOKES);

        assert!(zone.contains_angle(angle_0, SPOKES), "0° should be inside 270-90° (wraparound)");
        assert!(zone.contains_angle(angle_45, SPOKES), "45° should be inside 270-90°");
        assert!(zone.contains_angle(angle_90, SPOKES), "90° should be inside 270-90°");
        assert!(zone.contains_angle(angle_270, SPOKES), "270° should be inside 270-90°");
        assert!(zone.contains_angle(angle_315, SPOKES), "315° should be inside 270-90°");
        assert!(!zone.contains_angle(angle_180, SPOKES), "180° should be outside 270-90°");
    }

    #[test]
    fn test_guard_zone_negative_angles() {
        // Zone from -90° to 0° (port quarter to dead ahead)
        // -90° = 270° in spoke coordinates
        let zone = make_guard_zone(-90.0, 0.0, 100, 500);

        let angle_neg45 = SpokeAngle::from_radians((-45.0_f64).to_radians(), SPOKES);
        let angle_0 = SpokeAngle::from_radians(0.0, SPOKES);
        let angle_45 = SpokeAngle::from_radians(45.0_f64.to_radians(), SPOKES);

        assert!(zone.contains_angle(angle_neg45, SPOKES), "-45° should be inside -90°..0°");
        assert!(zone.contains_angle(angle_0, SPOKES), "0° should be inside -90°..0°");
        assert!(!zone.contains_angle(angle_45, SPOKES), "45° should be outside -90°..0°");
    }

    // =========================================================================
    // Guard Zone Contains (bearing + heading + range)
    // =========================================================================

    #[test]
    fn test_guard_zone_contains_with_zero_heading() {
        // Zone from 0° to 90° relative to bow, range 100-500 pixels
        let zone = make_guard_zone(0.0, 90.0, 100, 500);
        let heading = SpokeHeading::zero(); // Ship heading North

        // Bearing 45° geographic with heading 0° = 45° relative to bow
        let bearing = SpokeBearing::from_radians(45.0_f64.to_radians(), SPOKES);
        assert!(
            zone.contains(bearing, 300, SPOKES, heading),
            "bearing 45° with heading 0° should be inside zone 0-90° at range 300"
        );

        // Bearing 180° geographic with heading 0° = 180° relative to bow (astern)
        let bearing = SpokeBearing::from_radians(180.0_f64.to_radians(), SPOKES);
        assert!(
            !zone.contains(bearing, 300, SPOKES, heading),
            "bearing 180° with heading 0° should be outside zone 0-90°"
        );
    }

    #[test]
    fn test_guard_zone_contains_with_nonzero_heading() {
        // Zone from 0° to 90° relative to bow, range 100-500 pixels
        let zone = make_guard_zone(0.0, 90.0, 100, 500);

        // Ship heading 90° (East)
        let heading = SpokeHeading::from_radians(90.0_f64.to_radians(), SPOKES);

        // Bearing 90° geographic with heading 90° = 0° relative to bow (dead ahead)
        let bearing = SpokeBearing::from_radians(90.0_f64.to_radians(), SPOKES);
        assert!(
            zone.contains(bearing, 300, SPOKES, heading),
            "bearing 90° with heading 90° should be dead ahead (inside 0-90°)"
        );

        // Bearing 180° geographic with heading 90° = 90° relative to bow (starboard)
        let bearing = SpokeBearing::from_radians(180.0_f64.to_radians(), SPOKES);
        assert!(
            zone.contains(bearing, 300, SPOKES, heading),
            "bearing 180° with heading 90° should be starboard (inside 0-90°)"
        );

        // Bearing 0° geographic with heading 90° = -90° (270°) relative to bow (port)
        let bearing = SpokeBearing::from_radians(0.0, SPOKES);
        assert!(
            !zone.contains(bearing, 300, SPOKES, heading),
            "bearing 0° with heading 90° should be port (outside 0-90°)"
        );
    }

    #[test]
    fn test_guard_zone_contains_range_boundary() {
        // Use a normal angle range (not full circle which is a special case)
        let zone = make_guard_zone(0.0, 180.0, 100, 500);
        let heading = SpokeHeading::zero();
        let bearing = SpokeBearing::from_radians(45.0_f64.to_radians(), SPOKES);

        assert!(zone.contains(bearing, 100, SPOKES, heading), "inner boundary should be inside");
        assert!(zone.contains(bearing, 500, SPOKES, heading), "outer boundary should be inside");
        assert!(!zone.contains(bearing, 99, SPOKES, heading), "just before inner should be outside");
        assert!(!zone.contains(bearing, 501, SPOKES, heading), "just after outer should be outside");
    }

    // =========================================================================
    // Blob Detection with Guard Zones
    // =========================================================================

    #[test]
    fn test_blob_detection_target_inside_range() {
        // Guard zone: 0-90° relative to bow, range 100-500 pixels
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        // Target at bearing 45° (spoke ~256), range 300 pixels
        // With zero heading, bearing 45° = 45° relative to bow (inside zone)
        let heading = SpokeHeading::zero();
        let target_spoke = (45.0 / 360.0 * SPOKES as f64) as u32; // ~256

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            300, // Inside range 100-500
            heading,
        );

        assert!(
            !blobs.is_empty(),
            "Target at range 300 (inside 100-500) should be detected"
        );
    }

    #[test]
    fn test_blob_detection_target_outside_range_too_close() {
        // Guard zone: 0-90° relative to bow, range 100-500 pixels
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        let heading = SpokeHeading::zero();
        let target_spoke = (45.0 / 360.0 * SPOKES as f64) as u32;

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            50, // Too close, outside range 100-500
            heading,
        );

        assert!(
            blobs.is_empty(),
            "Target at range 50 (outside 100-500) should NOT be detected"
        );
    }

    #[test]
    fn test_blob_detection_target_outside_range_too_far() {
        // Guard zone: 0-90° relative to bow, range 100-500 pixels
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        let heading = SpokeHeading::zero();
        let target_spoke = (45.0 / 360.0 * SPOKES as f64) as u32;

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            600, // Too far, outside range 100-500
            heading,
        );

        assert!(
            blobs.is_empty(),
            "Target at range 600 (outside 100-500) should NOT be detected"
        );
    }

    #[test]
    fn test_blob_detection_target_at_inner_boundary() {
        // Guard zone: 0-90° relative to bow, range 100-500 pixels
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        let heading = SpokeHeading::zero();
        let target_spoke = (45.0 / 360.0 * SPOKES as f64) as u32;

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            100, // At inner boundary
            heading,
        );

        assert!(
            !blobs.is_empty(),
            "Target at range 100 (inner boundary) should be detected"
        );
    }

    #[test]
    fn test_blob_detection_target_at_outer_boundary() {
        // Guard zone: 0-90° relative to bow, range 100-500 pixels
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        let heading = SpokeHeading::zero();
        let target_spoke = (45.0 / 360.0 * SPOKES as f64) as u32;

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            500, // At outer boundary
            heading,
        );

        assert!(
            !blobs.is_empty(),
            "Target at range 500 (outer boundary) should be detected"
        );
    }

    #[test]
    fn test_blob_detection_target_outside_angle() {
        // Guard zone: 0-90° relative to bow
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        let heading = SpokeHeading::zero();
        // Target at bearing 180° (astern) - outside the 0-90° zone
        let target_spoke = (180.0 / 360.0 * SPOKES as f64) as u32; // ~1024

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            300, // Inside range, but wrong angle
            heading,
        );

        assert!(
            blobs.is_empty(),
            "Target at bearing 180° should NOT be detected in 0-90° zone"
        );
    }

    #[test]
    fn test_blob_detection_with_heading_offset() {
        // Guard zone: 0-90° relative to bow (straight ahead to starboard)
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        // Ship heading 180° (South)
        let heading = SpokeHeading::from_radians(180.0_f64.to_radians(), SPOKES);

        // Geographic bearing 180° with heading 180° = 0° relative to bow (dead ahead)
        // This should be inside the 0-90° zone
        let target_spoke = (180.0 / 360.0 * SPOKES as f64) as u32;

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            300,
            heading,
        );

        assert!(
            !blobs.is_empty(),
            "Target dead ahead (geo 180° with heading 180°) should be detected in 0-90° zone"
        );
    }

    #[test]
    fn test_blob_detection_heading_makes_target_outside() {
        // Guard zone: 0-90° relative to bow
        let mut detector = make_detector_with_zone(0.0, 90.0, 100, 500);

        // Ship heading 180° (South)
        let heading = SpokeHeading::from_radians(180.0_f64.to_radians(), SPOKES);

        // Geographic bearing 0° (North) with heading 180° (South) = 180° relative to bow (astern)
        // This should be OUTSIDE the 0-90° zone
        let target_spoke = 0;

        let blobs = simulate_target(
            &mut detector,
            target_spoke,
            target_spoke + 10,
            300,
            heading,
        );

        assert!(
            blobs.is_empty(),
            "Target astern (geo 0° with heading 180°) should NOT be detected in 0-90° zone"
        );
    }

    #[test]
    fn test_guard_zone_disabled_no_detection() {
        let mut detector = make_detector_with_zone(0.0, 360.0, 0, 1000);
        detector.guard_zones[0].enabled = false;

        let heading = SpokeHeading::zero();
        let blobs = simulate_target(&mut detector, 100, 110, 300, heading);

        assert!(
            blobs.is_empty(),
            "No targets should be detected when guard zone is disabled"
        );
    }

    // =========================================================================
    // Edge Cases
    // =========================================================================

    #[test]
    fn test_guard_zone_full_circle() {
        // Full circle zone: use wraparound from 180° to 179° (almost full circle)
        // This creates a zone that wraps all the way around, covering 359°
        // Note: 0° to 360° or -180° to 180° normalize to the same value and create empty zones
        let zone = make_guard_zone(180.0, 179.0, 100, 500);
        let heading = SpokeHeading::zero();

        for angle_deg in [0, 45, 90, 135, 180, 225, 270, 315] {
            let bearing = SpokeBearing::from_radians((angle_deg as f64).to_radians(), SPOKES);
            assert!(
                zone.contains(bearing, 300, SPOKES, heading),
                "Full circle zone (180 to 179) should contain bearing {}°",
                angle_deg
            );
        }
    }

    #[test]
    fn test_guard_zone_very_narrow_angle() {
        // Very narrow zone: just a few degrees
        let zone = make_guard_zone(0.0, 5.0, 100, 500);
        let heading = SpokeHeading::zero();

        let bearing_2 = SpokeBearing::from_radians(2.0_f64.to_radians(), SPOKES);
        let bearing_10 = SpokeBearing::from_radians(10.0_f64.to_radians(), SPOKES);

        assert!(
            zone.contains(bearing_2, 300, SPOKES, heading),
            "2° should be inside 0-5° zone"
        );
        assert!(
            !zone.contains(bearing_10, 300, SPOKES, heading),
            "10° should be outside 0-5° zone"
        );
    }

    #[test]
    fn test_guard_zone_very_narrow_range() {
        // Very narrow range zone - use a normal angle range
        let zone = make_guard_zone(0.0, 180.0, 200, 210);
        let heading = SpokeHeading::zero();
        let bearing = SpokeBearing::from_radians(45.0_f64.to_radians(), SPOKES);

        assert!(zone.contains(bearing, 200, SPOKES, heading), "200 should be inside 200-210");
        assert!(zone.contains(bearing, 205, SPOKES, heading), "205 should be inside 200-210");
        assert!(zone.contains(bearing, 210, SPOKES, heading), "210 should be inside 200-210");
        assert!(!zone.contains(bearing, 199, SPOKES, heading), "199 should be outside 200-210");
        assert!(!zone.contains(bearing, 211, SPOKES, heading), "211 should be outside 200-210");
    }
}
