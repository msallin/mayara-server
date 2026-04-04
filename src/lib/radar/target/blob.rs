//! Blob detection for radar target tracking.
//!
//! This module detects contiguous groups of strong pixels (blobs) in radar spokes
//! and identifies those that meet ship size constraints. All blobs are sent to the
//! tracker which decides whether to track them based on:
//! - Guard zone presence (automatic acquisition)
//! - Existing tracked target proximity (continue tracking)
//! - MARPA (manual acquisition via user click)
//! - DopplerAutoTrack (automatic acquisition of Doppler-colored targets)

use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;

use crate::config::GuardZone;
use crate::protos::RadarMessage::radar_message::Spoke;

/// Default minimum pixel intensity to be considered part of a blob (1/3 of max 15)
/// This is overridden by legend.medium_return which varies per radar brand.
const DEFAULT_BLOB_THRESHOLD: u8 = 5;

/// Minimum ship size in meters
pub const MIN_TARGET_SIZE_M: f64 = 5.0;

/// Maximum ship size in meters
pub const MAX_TARGET_SIZE_M: f64 = 600.0;

/// A single pixel belonging to a blob
#[derive(Clone, Debug)]
struct BlobPixel {
    spoke: u16,
    pixel: usize,
    #[allow(dead_code)] // May be useful for intensity-weighted center calculation
    intensity: u8,
}

/// A blob that is still being built as spokes arrive
struct BlobInProgress {
    #[allow(dead_code)] // Useful for debugging
    id: u32,
    pixels: Vec<BlobPixel>,
    /// Spatial index: pixel positions by spoke for O(1) adjacency lookup
    /// Key: spoke number, Value: sorted list of pixel indices on that spoke
    pixels_by_spoke: HashMap<u16, Vec<usize>>,
    last_spoke_with_addition: u16,
    min_spoke: u16,
    max_spoke: u16,
    min_pixel: usize,
    max_pixel: usize,
}

impl BlobInProgress {
    fn new(id: u32, pixel: BlobPixel) -> Self {
        let spoke = pixel.spoke;
        let pixel_idx = pixel.pixel;
        let mut pixels_by_spoke = HashMap::new();
        pixels_by_spoke.insert(spoke, vec![pixel_idx]);

        BlobInProgress {
            id,
            min_spoke: spoke,
            max_spoke: spoke,
            min_pixel: pixel_idx,
            max_pixel: pixel_idx,
            last_spoke_with_addition: spoke,
            pixels: vec![pixel],
            pixels_by_spoke,
        }
    }

    fn add_pixel(&mut self, pixel: BlobPixel, current_spoke: u16) {
        // Update spatial index
        self.pixels_by_spoke
            .entry(pixel.spoke)
            .or_insert_with(Vec::new)
            .push(pixel.pixel);

        // Update bounds
        self.min_pixel = self.min_pixel.min(pixel.pixel);
        self.max_pixel = self.max_pixel.max(pixel.pixel);
        self.min_spoke = self.min_spoke.min(pixel.spoke);
        self.max_spoke = self.max_spoke.max(pixel.spoke);
        self.last_spoke_with_addition = current_spoke;
        self.pixels.push(pixel);
    }

    /// Merge another blob into this one
    fn merge(&mut self, other: BlobInProgress, current_spoke: u16) {
        // Merge spatial index
        for (spoke, pixels) in other.pixels_by_spoke {
            self.pixels_by_spoke
                .entry(spoke)
                .or_insert_with(Vec::new)
                .extend(pixels);
        }

        // Merge bounds
        self.min_pixel = self.min_pixel.min(other.min_pixel);
        self.max_pixel = self.max_pixel.max(other.max_pixel);
        self.min_spoke = self.min_spoke.min(other.min_spoke);
        self.max_spoke = self.max_spoke.max(other.max_spoke);
        self.last_spoke_with_addition = current_spoke;
        self.pixels.extend(other.pixels);
    }

    /// Check if a pixel at (spoke, pixel_idx) is adjacent to any pixel in this blob
    /// Uses spatial index for O(1) average case instead of O(n)
    fn is_adjacent_to(&self, spoke: u16, pixel_idx: usize, spokes_per_revolution: u16) -> bool {
        // Get the three spokes we need to check (prev, current, next)
        let prev_spoke = if spoke == 0 {
            spokes_per_revolution - 1
        } else {
            spoke - 1
        };
        let next_spoke = (spoke + 1) % spokes_per_revolution;

        // Check each relevant spoke
        for &check_spoke in &[prev_spoke, spoke, next_spoke] {
            if let Some(pixels) = self.pixels_by_spoke.get(&check_spoke) {
                // Check if any pixel on this spoke is adjacent (within 1 pixel distance)
                for &p in pixels {
                    let diff = if p > pixel_idx {
                        p - pixel_idx
                    } else {
                        pixel_idx - p
                    };
                    if diff <= 1 {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn touches_spoke(&self, spoke: u16, spokes_per_revolution: u16) -> bool {
        let prev_spoke = if spoke == 0 {
            spokes_per_revolution - 1
        } else {
            spoke - 1
        };
        let next_spoke = (spoke + 1) % spokes_per_revolution;

        self.pixels_by_spoke.contains_key(&spoke)
            || self.pixels_by_spoke.contains_key(&prev_spoke)
            || self.pixels_by_spoke.contains_key(&next_spoke)
    }
}

/// A completed blob with contour information
#[derive(Clone)]
pub struct CompletedBlob {
    pub contour: Vec<(u16, usize)>,
    /// All pixels in the blob (for debug visualization)
    pub all_pixels: Vec<(u16, usize)>,
    pub center_spoke: u16,
    pub center_pixel: usize,
    pub size_meters: f64,
    /// Which guard zones contain this blob's center (1 and/or 2), empty if none
    pub in_guard_zones: Vec<u8>,
}

/// Internal representation of a guard zone in spoke/pixel coordinates
#[derive(Clone, Debug)]
struct GuardZoneInternal {
    /// Guard zone number (1 or 2)
    zone_id: u8,
    /// Start angle in spokes
    start_spoke: u16,
    /// End angle in spokes
    end_spoke: u16,
    /// Inner distance in pixels
    start_pixel: usize,
    /// Outer distance in pixels
    end_pixel: usize,
}

/// Blob detector that processes spokes and identifies targets
pub struct BlobDetector {
    spokes_per_revolution: u16,
    /// Minimum pixel intensity to be considered part of a blob (from legend.medium_return)
    threshold: u8,
    next_blob_id: u32,
    active_blobs: Vec<BlobInProgress>,
    spoke_buffer: VecDeque<Spoke>,
    current_range: u32,
    current_spoke_len: usize,
    /// Cached guard zone configs for refresh on range change
    guard_zone_1: Option<GuardZone>,
    guard_zone_2: Option<GuardZone>,
    /// Active guard zones in spoke/pixel coordinates
    guard_zones: Vec<GuardZoneInternal>,
}

impl BlobDetector {
    pub fn new(spokes_per_revolution: u16, threshold: u8) -> Self {
        let threshold = if threshold > 0 {
            threshold
        } else {
            DEFAULT_BLOB_THRESHOLD
        };
        BlobDetector {
            spokes_per_revolution,
            threshold,
            next_blob_id: 0,
            active_blobs: Vec::new(),
            spoke_buffer: VecDeque::new(),
            current_range: 0,
            current_spoke_len: 0,
            guard_zone_1: None,
            guard_zone_2: None,
            guard_zones: Vec::new(),
        }
    }

    /// Set guard zone 1 config (call when control changes)
    pub fn set_guard_zone_1(&mut self, zone: Option<GuardZone>) {
        self.guard_zone_1 = zone;
        self.refresh_guard_zones();
    }

    /// Set guard zone 2 config (call when control changes)
    pub fn set_guard_zone_2(&mut self, zone: Option<GuardZone>) {
        self.guard_zone_2 = zone;
        self.refresh_guard_zones();
    }

    /// Refresh guard zones from cached config (call when range/spoke_len changes)
    fn refresh_guard_zones(&mut self) {
        if self.current_range == 0 || self.current_spoke_len == 0 {
            if !self.guard_zones.is_empty() {
                self.guard_zones.clear();
            }
            return;
        }

        let meters_per_pixel = self.current_range as f64 / self.current_spoke_len as f64;

        // Build new guard zones
        let mut new_zones = Vec::new();
        for (zone_id, zone_opt) in [(1u8, &self.guard_zone_1), (2u8, &self.guard_zone_2)] {
            if let Some(zone) = zone_opt {
                if !zone.enabled {
                    continue;
                }

                // Convert angles from radians to spokes
                // Guard zones are head-relative (0 = forward)
                let start_spoke = ((zone.start_angle / (2.0 * PI))
                    * self.spokes_per_revolution as f64) as u16
                    % self.spokes_per_revolution;
                let end_spoke = ((zone.end_angle / (2.0 * PI)) * self.spokes_per_revolution as f64)
                    as u16
                    % self.spokes_per_revolution;

                // Convert distances from meters to pixels
                let start_pixel = (zone.start_distance / meters_per_pixel) as usize;
                let end_pixel = (zone.end_distance / meters_per_pixel) as usize;

                new_zones.push(GuardZoneInternal {
                    zone_id,
                    start_spoke,
                    end_spoke,
                    start_pixel,
                    end_pixel,
                });
            }
        }

        // Only update and log if zones changed
        let changed = new_zones.len() != self.guard_zones.len()
            || new_zones
                .iter()
                .zip(self.guard_zones.iter())
                .any(|(new, old)| {
                    new.zone_id != old.zone_id
                        || new.start_spoke != old.start_spoke
                        || new.end_spoke != old.end_spoke
                        || new.start_pixel != old.start_pixel
                        || new.end_pixel != old.end_pixel
                });

        if changed {
            for gz in &new_zones {
                log::debug!(
                    "Guard zone {}: spokes {}-{}, pixels {}-{}",
                    gz.zone_id,
                    gz.start_spoke,
                    gz.end_spoke,
                    gz.start_pixel,
                    gz.end_pixel
                );
            }
            self.guard_zones = new_zones;
        }
    }

    /// Check which guard zones contain a given spoke/pixel position
    fn check_guard_zones(&self, spoke: u16, pixel: usize) -> Vec<u8> {
        let mut zones = Vec::new();

        for gz in &self.guard_zones {
            // Check pixel (distance) is within range
            if pixel < gz.start_pixel || pixel > gz.end_pixel {
                continue;
            }

            // Check spoke (angle) is within range, handling wraparound
            let in_angle = if gz.start_spoke <= gz.end_spoke {
                // Normal case: start < end
                spoke >= gz.start_spoke && spoke <= gz.end_spoke
            } else {
                // Wraparound case: zone spans 0
                spoke >= gz.start_spoke || spoke <= gz.end_spoke
            };

            if in_angle {
                zones.push(gz.zone_id);
            }
        }

        zones
    }

    /// Calculate the physical size of a blob in meters
    fn calculate_size(&self, blob: &BlobInProgress) -> f64 {
        if self.current_range == 0 || self.current_spoke_len == 0 {
            return 0.0;
        }

        let meters_per_pixel = self.current_range as f64 / self.current_spoke_len as f64;

        // Radial extent
        let radial_extent = (blob.max_pixel - blob.min_pixel + 1) as f64 * meters_per_pixel;

        // Angular extent (at average distance)
        let avg_distance = (blob.min_pixel + blob.max_pixel) as f64 / 2.0 * meters_per_pixel;
        let spoke_extent = if blob.max_spoke >= blob.min_spoke {
            blob.max_spoke - blob.min_spoke + 1
        } else {
            // Wraparound
            self.spokes_per_revolution - blob.min_spoke + blob.max_spoke + 1
        };
        let angular_extent =
            avg_distance * (spoke_extent as f64 * 2.0 * PI / self.spokes_per_revolution as f64);

        // Use larger dimension as "size"
        radial_extent.max(angular_extent)
    }

    /// Calculate the contour (edge pixels) of a blob
    /// Uses the blob's spatial index for O(1) neighbor lookups
    fn calculate_contour(&self, blob: &BlobInProgress) -> Vec<(u16, usize)> {
        blob.pixels
            .iter()
            .filter(|p| {
                let prev_spoke = if p.spoke == 0 {
                    self.spokes_per_revolution - 1
                } else {
                    p.spoke - 1
                };
                let next_spoke = (p.spoke + 1) % self.spokes_per_revolution;

                // Check 4-neighbors using spatial index
                // A pixel is on the contour if any neighbor is missing
                let neighbors = [
                    (p.spoke, p.pixel.wrapping_sub(1)), // inner
                    (p.spoke, p.pixel + 1),             // outer
                    (prev_spoke, p.pixel),              // ccw
                    (next_spoke, p.pixel),              // cw
                ];

                neighbors.iter().any(|(spoke, pixel)| {
                    !blob
                        .pixels_by_spoke
                        .get(spoke)
                        .map(|pixels| pixels.contains(pixel))
                        .unwrap_or(false)
                })
            })
            .map(|p| (p.spoke, p.pixel))
            .collect()
    }

    /// Process a single spoke and return any completed blobs
    pub fn process_spoke(&mut self, spoke: &Spoke) -> Vec<CompletedBlob> {
        // Update range and spoke length if changed, then refresh guard zones
        let spoke_len = spoke.data.len();
        let range_changed = spoke.range != 0 && spoke.range != self.current_range;
        let spoke_len_changed = spoke_len != 0 && spoke_len != self.current_spoke_len;

        if range_changed {
            self.current_range = spoke.range;
            log::debug!("BlobDetector: range updated to {}m", self.current_range);
        }
        if spoke_len_changed {
            self.current_spoke_len = spoke_len;
        }
        if range_changed || spoke_len_changed {
            self.refresh_guard_zones();
        }

        // Use spoke.angle (head-relative) for guard zone checks since guard zones
        // are defined relative to boat heading, not true north
        let spoke_angle = spoke.angle as u16 % self.spokes_per_revolution;

        // Find strong pixels
        let mut strong_pixels: Vec<BlobPixel> = Vec::new();
        for (pixel_idx, &intensity) in spoke.data.iter().enumerate() {
            if intensity >= self.threshold {
                strong_pixels.push(BlobPixel {
                    spoke: spoke_angle,
                    pixel: pixel_idx,
                    intensity,
                });
            }
        }

        // Process each strong pixel - use spatial index for O(1) adjacency lookup
        for pixel in strong_pixels {
            // Find which blobs this pixel is adjacent to using spatial index
            let mut adjacent_blob_indices: Vec<usize> = Vec::new();
            for (idx, blob) in self.active_blobs.iter().enumerate() {
                if blob.is_adjacent_to(pixel.spoke, pixel.pixel, self.spokes_per_revolution) {
                    adjacent_blob_indices.push(idx);
                }
            }

            match adjacent_blob_indices.len() {
                0 => {
                    // Start new blob
                    let id = self.next_blob_id;
                    self.next_blob_id += 1;
                    self.active_blobs.push(BlobInProgress::new(id, pixel));
                }
                1 => {
                    // Add to existing blob
                    self.active_blobs[adjacent_blob_indices[0]].add_pixel(pixel, spoke_angle);
                }
                _ => {
                    // Merge multiple blobs - use dedicated merge method
                    adjacent_blob_indices.sort_unstable();
                    adjacent_blob_indices.reverse();

                    // Remove all but the target (lowest index) and merge them
                    let target_idx = *adjacent_blob_indices.last().unwrap();
                    for &idx in adjacent_blob_indices
                        .iter()
                        .take(adjacent_blob_indices.len() - 1)
                    {
                        let removed = self.active_blobs.remove(idx);
                        self.active_blobs[target_idx].merge(removed, spoke_angle);
                    }
                    self.active_blobs[target_idx].add_pixel(pixel, spoke_angle);
                }
            }
        }

        // Check for completed blobs (not extended on this spoke)
        let mut completed: Vec<CompletedBlob> = Vec::new();
        let mut to_remove: Vec<usize> = Vec::new();

        for (idx, blob) in self.active_blobs.iter().enumerate() {
            if blob.last_spoke_with_addition != spoke_angle {
                // Check if this blob is truly done (no pixels on adjacent spokes still coming)
                let prev_spoke = if spoke_angle == 0 {
                    self.spokes_per_revolution - 1
                } else {
                    spoke_angle - 1
                };
                if blob.last_spoke_with_addition != prev_spoke {
                    // Blob is complete
                    let size = self.calculate_size(blob);
                    log::debug!(
                        "BlobDetector: completed blob with {} pixels, size {:.1}m (valid: {})",
                        blob.pixels.len(),
                        size,
                        size >= MIN_TARGET_SIZE_M && size <= MAX_TARGET_SIZE_M
                    );
                    if size >= MIN_TARGET_SIZE_M && size <= MAX_TARGET_SIZE_M {
                        let contour = self.calculate_contour(blob);
                        let all_pixels: Vec<(u16, usize)> =
                            blob.pixels.iter().map(|p| (p.spoke, p.pixel)).collect();
                        // Calculate center spoke, handling wraparound
                        let center_spoke = if blob.max_spoke >= blob.min_spoke {
                            // Normal case: no wraparound
                            ((blob.min_spoke as u32 + blob.max_spoke as u32) / 2) as u16
                        } else {
                            // Wraparound case: blob spans spoke 0
                            // Add spokes_per_revolution to max_spoke for averaging, then normalize
                            let adjusted_max =
                                blob.max_spoke as u32 + self.spokes_per_revolution as u32;
                            let center = (blob.min_spoke as u32 + adjusted_max) / 2;
                            (center % self.spokes_per_revolution as u32) as u16
                        };
                        let center_pixel = (blob.min_pixel + blob.max_pixel) / 2;
                        let in_guard_zones = self.check_guard_zones(center_spoke, center_pixel);
                        completed.push(CompletedBlob {
                            contour,
                            all_pixels,
                            center_spoke,
                            center_pixel,
                            size_meters: size,
                            in_guard_zones,
                        });
                    }
                    to_remove.push(idx);
                }
            }
        }

        // Remove completed blobs (in reverse order to preserve indices)
        to_remove.sort_unstable();
        to_remove.reverse();
        for idx in to_remove {
            self.active_blobs.remove(idx);
        }

        // Buffer this spoke for contour drawing and ready-check
        self.spoke_buffer.push_back(spoke.clone());

        // Limit buffer to 1/4 revolution to prevent unbounded growth
        let max_buffer_size = (self.spokes_per_revolution / 4) as usize;
        while self.spoke_buffer.len() > max_buffer_size {
            self.spoke_buffer.pop_front();
        }

        completed
    }

    /// Get spokes that are ready to be sent (no active blobs touch them)
    pub fn get_ready_spokes(&mut self) -> Vec<Spoke> {
        let mut ready = Vec::new();

        while let Some(oldest) = self.spoke_buffer.front() {
            let spoke_angle = oldest.angle as u16 % self.spokes_per_revolution;

            // Check if any active blob touches this spoke (uses head-relative angle)
            let blob_touches = self
                .active_blobs
                .iter()
                .any(|b| b.touches_spoke(spoke_angle, self.spokes_per_revolution));

            if blob_touches {
                break;
            }

            ready.push(self.spoke_buffer.pop_front().unwrap());
        }

        ready
    }
}
