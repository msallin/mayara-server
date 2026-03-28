//! Exclusion zone mask for stationary radar installations.
//!
//! Exclusion zones are areas where radar returns are suppressed (set to transparent).
//! The mask is precomputed when the range changes to ensure O(1) lookup during spoke processing.

use std::f64::consts::PI;

use crate::config::{ExclusionRect, ExclusionZone};

/// Internal representation of a sector exclusion zone in spoke/pixel coordinates
#[derive(Debug, Clone)]
pub struct ExclusionZoneInternal {
    pub start_spoke: u16,
    pub end_spoke: u16,
    pub start_pixel: usize,
    pub end_pixel: usize,
}

/// Internal representation of a rectangular exclusion zone in meters
#[derive(Debug, Clone)]
pub struct ExclusionRectInternal {
    pub north: f64,
    pub south: f64,
    pub east: f64,
    pub west: f64,
}

/// Precomputed exclusion mask for efficient spoke processing.
///
/// Uses a bitmask to store exclusion state per pixel, allowing O(1) lookup.
/// The mask is rebuilt when range changes or exclusion zones are modified.
pub struct ExclusionMask {
    /// Bitmask: 1 bit per pixel (0=not excluded, 1=excluded)
    /// Indexed by [spoke][pixel / 8], bit = pixel % 8
    mask: Vec<Vec<u8>>,
    spokes: u16,
    pixels_per_spoke: usize,
}

impl ExclusionMask {
    /// Create a new exclusion mask from zone configurations.
    ///
    /// # Arguments
    /// * `zones` - Active sector exclusion zones in internal coordinates
    /// * `rects` - Active rectangular exclusion zones in meters
    /// * `spokes` - Number of spokes per revolution
    /// * `pixels` - Number of pixels per spoke
    /// * `range_meters` - Current radar range in meters
    pub fn new(
        zones: &[ExclusionZoneInternal],
        rects: &[ExclusionRectInternal],
        spokes: u16,
        pixels: usize,
        range_meters: u32,
    ) -> Self {
        // Each byte stores 8 pixels (1 bit each)
        let bytes_per_spoke = (pixels + 7) / 8;
        let mut mask = vec![vec![0u8; bytes_per_spoke]; spokes as usize];

        // Build the mask for all sector zones
        for zone in zones {
            for spoke in 0..spokes {
                // Check spoke (angle) is within range, handling wraparound
                let in_angle = if zone.start_spoke <= zone.end_spoke {
                    spoke >= zone.start_spoke && spoke <= zone.end_spoke
                } else {
                    // Wraparound case: zone spans 0
                    spoke >= zone.start_spoke || spoke <= zone.end_spoke
                };

                if !in_angle {
                    continue;
                }

                // Set bits for all pixels in the distance range
                let start = zone.start_pixel.min(pixels);
                let end = zone.end_pixel.min(pixels);
                for pixel in start..=end {
                    let byte_idx = pixel / 8;
                    let bit = pixel % 8;
                    mask[spoke as usize][byte_idx] |= 1 << bit;
                }
            }
        }

        // Build the mask for all rectangular zones
        // For each spoke, compute the pixel range that intersects the rectangle
        let meters_per_pixel = range_meters as f64 / pixels as f64;

        for rect in rects {
            for spoke in 0..spokes {
                // Angle for this spoke (0 = north, increasing clockwise)
                let angle = (spoke as f64 / spokes as f64) * 2.0 * PI;
                let sin_a = angle.sin(); // x component (east)
                let cos_a = angle.cos(); // y component (north)

                // Find where the ray intersects the rectangle boundaries
                // Ray: x = d * sin_a, y = d * cos_a
                // Rectangle: -west <= x <= east, -south <= y <= north

                // Compute distances to each boundary (if ray hits it)
                let mut d_min = 0.0_f64;
                let mut d_max = f64::INFINITY;

                // X boundaries (east/west)
                if sin_a.abs() > 1e-10 {
                    let d_east = rect.east / sin_a;
                    let d_west = -rect.west / sin_a;
                    let (d_enter, d_exit) = if sin_a > 0.0 {
                        (d_west, d_east)
                    } else {
                        (d_east, d_west)
                    };
                    d_min = d_min.max(d_enter);
                    d_max = d_max.min(d_exit);
                } else {
                    // Ray is vertical (north/south) - check if within east/west bounds
                    // At d=0, x=0 which is within [-west, east] if west >= 0 and east >= 0
                    if rect.west < 0.0 || rect.east < 0.0 {
                        continue; // No intersection
                    }
                }

                // Y boundaries (north/south)
                if cos_a.abs() > 1e-10 {
                    let d_north = rect.north / cos_a;
                    let d_south = -rect.south / cos_a;
                    let (d_enter, d_exit) = if cos_a > 0.0 {
                        (d_south, d_north)
                    } else {
                        (d_north, d_south)
                    };
                    d_min = d_min.max(d_enter);
                    d_max = d_max.min(d_exit);
                } else {
                    // Ray is horizontal (east/west) - check if within north/south bounds
                    if rect.north < 0.0 || rect.south < 0.0 {
                        continue; // No intersection
                    }
                }

                // Check if there's a valid intersection
                if d_max <= d_min || d_max <= 0.0 {
                    continue;
                }
                d_min = d_min.max(0.0);

                // Convert distances to pixel indices
                let start_pixel = (d_min / meters_per_pixel) as usize;
                let end_pixel = ((d_max / meters_per_pixel) as usize).min(pixels.saturating_sub(1));

                // Set bits for all pixels in range
                for pixel in start_pixel..=end_pixel {
                    let byte_idx = pixel / 8;
                    let bit = pixel % 8;
                    mask[spoke as usize][byte_idx] |= 1 << bit;
                }
            }
        }

        ExclusionMask {
            mask,
            spokes,
            pixels_per_spoke: pixels,
        }
    }

    /// Check if a pixel is excluded.
    #[inline]
    pub fn is_excluded(&self, spoke: u16, pixel: usize) -> bool {
        if spoke >= self.spokes || pixel >= self.pixels_per_spoke {
            return false;
        }

        let byte_idx = pixel / 8;
        let bit = pixel % 8;
        (self.mask[spoke as usize][byte_idx] >> bit) & 1 == 1
    }
}

/// Convert exclusion zone from config to internal spoke/pixel coordinates.
pub fn zone_to_internal(
    zone: &ExclusionZone,
    spokes_per_revolution: u16,
    range_meters: u32,
    spoke_len: usize,
) -> ExclusionZoneInternal {
    let meters_per_pixel = range_meters as f64 / spoke_len as f64;

    // Convert angles from radians to spokes
    let start_spoke =
        ((zone.start_angle / (2.0 * PI)) * spokes_per_revolution as f64) as i32;
    let start_spoke = start_spoke.rem_euclid(spokes_per_revolution as i32) as u16;

    let end_spoke = ((zone.end_angle / (2.0 * PI)) * spokes_per_revolution as f64) as i32;
    let end_spoke = end_spoke.rem_euclid(spokes_per_revolution as i32) as u16;

    // Convert distances from meters to pixels
    let start_pixel = (zone.start_distance / meters_per_pixel) as usize;
    let end_pixel = (zone.end_distance / meters_per_pixel).min(spoke_len as f64) as usize;

    ExclusionZoneInternal {
        start_spoke,
        end_spoke,
        start_pixel,
        end_pixel,
    }
}

/// Convert rectangular exclusion zone from config to internal format.
/// Returns None if the rect has invalid bounds.
pub fn rect_to_internal(rect: &ExclusionRect) -> ExclusionRectInternal {
    ExclusionRectInternal {
        north: rect.north,
        south: rect.south,
        east: rect.east,
        west: rect.west,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exclusion_mask_empty() {
        let mask = ExclusionMask::new(&[], &[], 360, 512, 1000);
        assert!(!mask.is_excluded(0, 0));
        assert!(!mask.is_excluded(180, 256));
    }

    #[test]
    fn test_exclusion_mask_simple_zone() {
        let zones = vec![ExclusionZoneInternal {
            start_spoke: 10,
            end_spoke: 20,
            start_pixel: 100,
            end_pixel: 200,
        }];
        let mask = ExclusionMask::new(&zones, &[], 360, 512, 1000);

        // Outside zone
        assert!(!mask.is_excluded(5, 150));
        assert!(!mask.is_excluded(15, 50));
        assert!(!mask.is_excluded(15, 250));

        // Inside zone
        assert!(mask.is_excluded(10, 100));
        assert!(mask.is_excluded(15, 150));
        assert!(mask.is_excluded(20, 200));

        // Boundary checks
        assert!(mask.is_excluded(10, 150)); // start spoke
        assert!(mask.is_excluded(20, 150)); // end spoke
        assert!(mask.is_excluded(15, 100)); // start pixel
        assert!(mask.is_excluded(15, 200)); // end pixel
    }

    #[test]
    fn test_exclusion_mask_wraparound() {
        // Zone that wraps around 0
        let zones = vec![ExclusionZoneInternal {
            start_spoke: 350,
            end_spoke: 10,
            start_pixel: 100,
            end_pixel: 200,
        }];
        let mask = ExclusionMask::new(&zones, &[], 360, 512, 1000);

        // Inside zone (before wrap)
        assert!(mask.is_excluded(350, 150));
        assert!(mask.is_excluded(355, 150));
        assert!(mask.is_excluded(359, 150));
        // Inside zone (after wrap)
        assert!(mask.is_excluded(0, 150));
        assert!(mask.is_excluded(5, 150));
        assert!(mask.is_excluded(10, 150));
        // Outside zone
        assert!(!mask.is_excluded(180, 150));
        assert!(!mask.is_excluded(11, 150));
        assert!(!mask.is_excluded(349, 150));
    }

    #[test]
    fn test_zone_to_internal() {
        let zone = ExclusionZone {
            start_angle: 0.0,
            end_angle: PI / 2.0, // 90 degrees
            start_distance: 100.0,
            end_distance: 500.0,
            enabled: true,
        };

        let internal = zone_to_internal(&zone, 360, 1000, 1000);

        assert_eq!(internal.start_spoke, 0);
        assert_eq!(internal.end_spoke, 90);
        assert_eq!(internal.start_pixel, 100);
        assert_eq!(internal.end_pixel, 500);
    }

    #[test]
    fn test_exclusion_mask_rectangle() {
        // Rectangle 100m north, 100m south, 100m east, 100m west from radar
        // With 1000m range and 1000 pixels, each pixel is 1m
        let rects = vec![ExclusionRectInternal {
            north: 100.0,
            south: 100.0,
            east: 100.0,
            west: 100.0,
        }];
        let mask = ExclusionMask::new(&[], &rects, 360, 1000, 1000);

        // Center (pixel 0) should be excluded
        assert!(mask.is_excluded(0, 0));
        assert!(mask.is_excluded(90, 0));
        assert!(mask.is_excluded(180, 0));
        assert!(mask.is_excluded(270, 0));

        // At 50m north (spoke 0, pixel ~50) should be excluded
        assert!(mask.is_excluded(0, 50));

        // At 150m north (spoke 0, pixel ~150) should NOT be excluded
        assert!(!mask.is_excluded(0, 150));

        // At 50m east (spoke 90, pixel ~50) should be excluded
        assert!(mask.is_excluded(90, 50));

        // At 50m south (spoke 180, pixel ~50) should be excluded
        assert!(mask.is_excluded(180, 50));

        // At 50m west (spoke 270, pixel ~50) should be excluded
        assert!(mask.is_excluded(270, 50));
    }

    #[test]
    fn test_rect_to_internal() {
        let rect = ExclusionRect {
            north: 200.0,
            south: 100.0,
            east: 150.0,
            west: 50.0,
            enabled: true,
        };

        let internal = rect_to_internal(&rect);

        assert_eq!(internal.north, 200.0);
        assert_eq!(internal.south, 100.0);
        assert_eq!(internal.east, 150.0);
        assert_eq!(internal.west, 50.0);
    }
}
