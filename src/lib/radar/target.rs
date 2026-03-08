#![allow(dead_code, unused_variables)]

use bitflags::bitflags;
use serde::Serialize;
use std::{
    cmp::{max, min},
    collections::HashMap,
    f64::consts::PI,
    sync::{Arc, RwLock},
};
use strum::{EnumIter, IntoEnumIterator};
use utoipa::ToSchema;

use kalman::{KalmanFilter, LocalPosition, Polar};
use ndarray::Array2;

use super::settings::{ControlError, ControlId};
use super::{GeoPosition, Legend, RadarInfo};
use crate::{navdata, protos::RadarMessage::radar_message::Spoke, radar::NAUTICAL_MILE_F64};

mod arpa;
mod kalman;
pub(crate) mod manager;

// ============================================================================
// Signal K API Types for Target Streaming
// ============================================================================

/// Signal K compatible target representation for API/WebSocket streaming
#[derive(Serialize, Clone, Debug, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct ArpaTargetApi {
    /// Target ID (unique within radar)
    pub id: usize,
    /// Current status: "tracking", "acquiring", or "lost"
    pub status: String,
    /// Target position relative to radar
    pub position: TargetPositionApi,
    /// Target motion (course and speed)
    pub motion: TargetMotionApi,
    /// Collision danger assessment
    pub danger: TargetDangerApi,
    /// How target was acquired: "auto" or "manual"
    pub acquisition: String,
}

/// Target position in the API format
#[derive(Serialize, Clone, Debug, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct TargetPositionApi {
    /// Bearing from radar in radians true [0, 2π)
    pub bearing: f64,
    /// Distance from radar in meters
    pub distance: f64,
    /// Latitude if available
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latitude: Option<f64>,
    /// Longitude if available
    #[serde(skip_serializing_if = "Option::is_none")]
    pub longitude: Option<f64>,
}

/// Target motion in the API format
#[derive(Serialize, Clone, Debug, ToSchema)]
pub struct TargetMotionApi {
    /// Course over ground in radians true [0, 2π)
    pub course: f64,
    /// Speed in m/s
    pub speed: f64,
}

/// Collision danger assessment in the API format
#[derive(Serialize, Clone, Debug, ToSchema)]
pub struct TargetDangerApi {
    /// Closest Point of Approach in meters
    pub cpa: f64,
    /// Time to CPA in seconds (negative = past)
    pub tcpa: f64,
}

const MIN_BLOB_PIXELS: usize = 1000; // minimum number of pixels for a valid blob
const MAX_BLOB_PIXELS: usize = 10000; // maximum blob size (radar interference protection)
const MAX_LOST_COUNT: i32 = 12; // number of sweeps that target can be missed before it is set to lost
const MAX_DETECTION_SPEED_KN: f64 = 40.;
const MIN_BLOB_RANGE: i32 = 4; // ignore blobs closer than this (main bang)

// Legacy constants - to be removed after full migration to blob-based detection
const MIN_CONTOUR_LENGTH: usize = 6;
const MAX_CONTOUR_LENGTH: usize = 2000;

pub const METERS_PER_DEGREE_LATITUDE: f64 = 60. * NAUTICAL_MILE_F64;
pub const KN_TO_MS: f64 = NAUTICAL_MILE_F64 / 3600.;
pub const MS_TO_KN: f64 = 3600. / NAUTICAL_MILE_F64;

const TODO_ROTATION_SPEED_MS: i32 = 2500;
const TODO_TARGET_AGE_TO_MIXER: u32 = 5;
const MAX_NUMBER_OF_TARGETS: usize = 100;

/// Guard zone configuration for target detection
/// Angles are stored in spokes (radar units), distances in pixels
#[derive(Debug, Clone)]
pub(crate) struct DetectionGuardZone {
    /// Start angle in spokes (0..spokes_per_revolution)
    start_angle: i32,
    /// End angle in spokes (0..spokes_per_revolution)
    end_angle: i32,
    /// Inner distance in pixels
    inner_range: i32,
    /// Outer distance in pixels
    outer_range: i32,
    /// Whether this guard zone is enabled for detection
    enabled: bool,
    /// Last scan time for each angle to avoid duplicate detections
    last_scan_time: Vec<u64>,
    // Original config values (for recalculation when pixels_per_meter changes)
    config_start_angle_rad: f64,
    config_end_angle_rad: f64,
    config_inner_range_m: f64,
    config_outer_range_m: f64,
    config_enabled: bool,
}

impl DetectionGuardZone {
    fn new(spokes_per_revolution: i32) -> Self {
        Self {
            start_angle: 0,
            end_angle: 0,
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
    fn update_from_config(
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
    fn recalculate(&mut self, spokes_per_revolution: i32, pixels_per_meter: f64) {
        self.enabled = self.config_enabled && pixels_per_meter > 0.0;

        if !self.enabled {
            return;
        }

        // Convert angles from radians to spokes
        let spokes_f64 = spokes_per_revolution as f64;
        self.start_angle = ((self.config_start_angle_rad / (2.0 * PI) * spokes_f64) as i32)
            .rem_euclid(spokes_per_revolution);
        self.end_angle = ((self.config_end_angle_rad / (2.0 * PI) * spokes_f64) as i32)
            .rem_euclid(spokes_per_revolution);

        // Convert distances from meters to pixels
        self.inner_range = (self.config_inner_range_m * pixels_per_meter).max(1.0) as i32;
        self.outer_range = (self.config_outer_range_m * pixels_per_meter) as i32;

        log::info!(
            "GuardZone configured: angles={}..{} spokes, range={}..{} pixels (ppm={})",
            self.start_angle,
            self.end_angle,
            self.inner_range,
            self.outer_range,
            pixels_per_meter
        );
    }

    /// Check if this zone has config that needs recalculation
    fn has_pending_config(&self) -> bool {
        self.config_enabled && !self.enabled
    }

    /// Check if an angle (in spokes) is within this guard zone
    /// The angle should be in relative coordinates (relative to ship heading)
    fn contains_angle(&self, relative_angle: i32, spokes_per_revolution: i32) -> bool {
        if !self.enabled {
            return false;
        }
        let angle = relative_angle.rem_euclid(spokes_per_revolution);
        if self.start_angle <= self.end_angle {
            angle >= self.start_angle && angle <= self.end_angle
        } else {
            // Zone wraps around 0
            angle >= self.start_angle || angle <= self.end_angle
        }
    }

    /// Check if a range (in pixels) is within this guard zone
    fn contains_range(&self, range: i32) -> bool {
        self.enabled && range >= self.inner_range && range <= self.outer_range
    }

    /// Check if a position is within the guard zone
    /// geographic_angle: the angle in geographic coordinates (0 = North)
    /// heading: the ship's heading in spokes (to convert geographic to relative)
    fn contains(
        &self,
        geographic_angle: i32,
        range: i32,
        spokes_per_revolution: i32,
        heading: i32,
    ) -> bool {
        // Convert geographic angle to relative angle by subtracting heading
        // Guard zone angles are stored relative to ship heading
        let relative_angle = (geographic_angle - heading).rem_euclid(spokes_per_revolution);
        self.contains_angle(relative_angle, spokes_per_revolution) && self.contains_range(range)
    }
}

///
/// The length of a degree longitude varies by the latitude,
/// the more north or south you get the shorter it becomes.
/// Since the earth is _nearly_ a sphere, the cosine function
/// is _very_ close.
///
pub fn meters_per_degree_longitude(lat: &f64) -> f64 {
    METERS_PER_DEGREE_LATITUDE * lat.to_radians().cos()
}

#[derive(Debug, Clone)]
pub(crate) struct ExtendedPosition {
    pub(crate) pos: GeoPosition,
    pub(crate) dlat_dt: f64, // m / sec
    pub(crate) dlon_dt: f64, // m / sec
    pub(crate) time: u64,    // millis
    pub(crate) speed_kn: f64,
    pub(crate) sd_speed_kn: f64, // standard deviation of the speed in knots
}

impl ExtendedPosition {
    pub(crate) fn new(
        pos: GeoPosition,
        dlat_dt: f64,
        dlon_dt: f64,
        time: u64,
        speed_kn: f64,
        sd_speed_kn: f64,
    ) -> Self {
        Self {
            pos,
            dlat_dt,
            dlon_dt,
            time,
            speed_kn,
            sd_speed_kn,
        }
    }
    fn empty() -> Self {
        Self::new(GeoPosition::new(0., 0.), 0., 0., 0, 0., 0.)
    }
}

// We try to find each target three times, with different conditions each time
#[derive(Debug, Clone, Copy, PartialEq, EnumIter)]
enum Pass {
    First,
    Second,
    Third,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum TargetStatus {
    Acquire0,    // Under acquisition, first seen, no contour yet
    Acquire1,    // Under acquisition, first contour found
    Acquire2,    // Under acquisition, speed and course known
    Acquire3,    // Under acquisition, speed and course known, next time active
    Active,      // Active target
    Lost,        // Lost target
    ForDeletion, // Target to be deleted
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum RefreshState {
    NotFound,
    Found,
    OutOfScope,
}

/*
Doppler states of the target.
A Doppler state of a target is an attribute of the target that determines the
search method for the target in the history array, according to the following
table:

x means don't care, bit0 is the above threshold bit, bit2 is the APPROACHING
bit, bit3 is the RECEDING bit.

                  bit0   bit2   bit3
ANY                  1      x      x
NO_DOPPLER           1      0      0
APPROACHING          1      1      0
RECEDING             1      0      1
ANY_DOPPLER          1      1      0   or
                     1      0      1
NOT_RECEDING         1      x      0
NOT_APPROACHING      1      0      x

ANY is typical non Dopper target
NOT_RECEDING and NOT_APPROACHING are only used to check countour length in the
transition of APPROACHING or RECEDING -> ANY ANY_DOPPLER is only used in the
search for targets and converted to APPROACHING or RECEDING in the first refresh
cycle

State transitions:
ANY -> APPROACHING or RECEDING  (not yet implemented)
APPROACHING or RECEDING -> ANY  (based on length of contours)
ANY_DOPPLER -> APPROACHING or RECEDING

*/
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Doppler {
    Any,            // any target above threshold
    NoDoppler,      // a target without a Doppler bit
    Approaching,    // Doppler approaching
    Receding,       // Doppler receding
    AnyDoppler,     // Approaching or Receding
    NotReceding,    // that is NoDoppler or Approaching
    NotApproaching, // that is NoDoppler or Receding
    AnyPlus,        // will also check bits that have been cleared
}

const FOUR_DIRECTIONS: [Polar; 4] = [
    Polar {
        angle: 0,
        r: 1,
        time: 0,
    },
    Polar {
        angle: 1,
        r: 0,
        time: 0,
    },
    Polar {
        angle: 0,
        r: -1,
        time: 0,
    },
    Polar {
        angle: -1,
        r: 0,
        time: 0,
    },
];

bitflags! {
    /// Represents a set of flags.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct HistoryPixel: u8 {
        /// The value `TARGET`, at bit position `7`.
        const TARGET = 0b10000000;
        /// The value `BACKUP`, at bit position `6`.
        const BACKUP = 0b01000000;
        /// The value `APPROACHING`, at bit position `5`.
        const APPROACHING = 0b00100000;
        /// The value `RECEDING`, at bit position `4`.
        const RECEDING = 0b00010000;
        /// The value `CONTOUR`, at bit position `3`.
        const CONTOUR = 0b00001000;
        /// The value `BLOB_EDGE`, at bit position `2` - marks edges of detected blobs for overlay
        const BLOB_EDGE = 0b00000100;

        /// The default value for a new one.
        const INITIAL = Self::TARGET.bits() | Self::BACKUP.bits();
        const NO_TARGET = !(Self::INITIAL.bits());
    }
}

impl HistoryPixel {
    fn new() -> Self {
        HistoryPixel::INITIAL
    }
}

#[derive(Debug, Clone)]
struct HistorySpoke {
    sweep: Vec<HistoryPixel>,
    time: u64,
    pos: GeoPosition,
}

#[derive(Debug, Clone)]
struct HistorySpokes {
    spokes: Box<Vec<HistorySpoke>>,
    stationary_layer: Option<Box<Array2<u8>>>,
}
#[derive(Debug, Clone)]
pub struct TargetBuffer {
    setup: TargetSetup,
    next_target_id: usize,
    history: HistorySpokes,
    targets: Arc<RwLock<HashMap<usize, ArpaTarget>>>,

    arpa_via_doppler: bool,

    m_clear_contours: bool,
    m_auto_learn_state: i32,

    // Average course
    course: f64,
    course_weight: u16,
    course_samples: u16,

    // If we have just received angle <n>
    // then we look for refreshed targets in <n + spokes_per_revolution/4> .. <n +
    // spokes_per_revolution / 2>
    scanned_angle: i32,
    // and we scan for new targets at <n + 3/4 * spokes_per_revolution> (SCAN_FOR_NEW_PERCENTAGE)
    refreshed_angle: i32,

    /// Shared target manager for dual radar coordination (None for single radar)
    shared_manager: Option<manager::SharedTargetManager>,
    /// Key identifying this radar
    radar_key: String,

    /// Guard zones for target detection
    guard_zones: [DetectionGuardZone; 2],

    /// Previous angle for detecting revolution completion
    prev_angle: usize,

    /// Current heading in spokes (updated each spoke, used for guard zone checks)
    current_heading: i32,

    /// Blobs currently being built as spokes arrive
    blobs_in_progress: Vec<BlobInProgress>,

    /// Broadcast sender for pushing target updates to SignalK stream
    sk_client_tx: Option<tokio::sync::broadcast::Sender<crate::stream::SignalKDelta>>,
}

const REFRESH_START_PERCENTAGE: i32 = 25;
const REFRESH_END_PERCENTAGE: i32 = 50;
const SCAN_FOR_NEW_PERCENTAGE: i32 = 75;

#[derive(Debug, Clone)]
pub(self) struct TargetSetup {
    key: String,
    spokes_per_revolution: i32,
    spokes_per_revolution_f64: f64,
    spoke_len: i32,
    have_doppler: bool,
    pixels_per_meter: f64,
    rotation_speed_ms: u32,
    stationary: bool,
}

/// A blob being incrementally built as spokes arrive
#[derive(Debug, Clone)]
struct BlobInProgress {
    /// Range values present on the last spoke that contributed to this blob
    /// Used to check adjacency with the next spoke
    last_spoke_ranges: Vec<i32>,
    /// The angle of the last spoke that contributed pixels
    last_angle: i32,
    /// Bounding box
    min_angle: i32,
    max_angle: i32,
    min_r: i32,
    max_r: i32,
    /// Total pixel count
    pixel_count: usize,
    /// Time when first pixel was seen
    start_time: u64,
    /// Own ship position when blob started
    start_pos: GeoPosition,
}

impl BlobInProgress {
    fn new(angle: i32, r: i32, time: u64, pos: GeoPosition) -> Self {
        Self {
            last_spoke_ranges: vec![r],
            last_angle: angle,
            min_angle: angle,
            max_angle: angle,
            min_r: r,
            max_r: r,
            pixel_count: 1,
            start_time: time,
            start_pos: pos,
        }
    }

    /// Add a pixel to this blob
    fn add_pixel(&mut self, angle: i32, r: i32) {
        self.max_angle = angle; // angle always increases as we process spokes
        self.min_r = min(self.min_r, r);
        self.max_r = max(self.max_r, r);
        self.pixel_count += 1;
    }

    /// Start a new spoke - clear last_spoke_ranges and set last_angle
    fn start_new_spoke(&mut self, angle: i32) {
        self.last_spoke_ranges.clear();
        self.last_angle = angle;
    }

    /// Check if a range value on the current spoke is adjacent to this blob
    /// (i.e., within 1 pixel of any range on the previous spoke)
    fn is_adjacent(&self, r: i32) -> bool {
        self.last_spoke_ranges
            .iter()
            .any(|&prev_r| (prev_r - r).abs() <= 1)
    }

    /// Calculate center position in polar coordinates
    fn center(&self) -> (i32, i32) {
        let center_angle = (self.min_angle + self.max_angle) / 2;
        let center_r = (self.min_r + self.max_r) / 2;
        (center_angle, center_r)
    }

    /// Check if blob meets minimum size requirements
    fn is_valid(&self) -> bool {
        self.pixel_count >= MIN_BLOB_PIXELS
            && self.pixel_count <= MAX_BLOB_PIXELS
            && self.min_r >= MIN_BLOB_RANGE
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Contour {
    pub(crate) length: i32,
    min_angle: i32,
    max_angle: i32,
    min_r: i32,
    max_r: i32,
    pub(crate) position: Polar,
    contour: Vec<Polar>,
}

#[derive(Debug, Clone)]
enum Error {
    RangeTooHigh,
    RangeTooLow,
    NoEchoAtStart,
    StartPointNotOnContour,
    BrokenContour,
    NoContourFound,
    AlreadyFound,
    NotFound,
    ContourLengthTooHigh,
    Lost,
    WeightedContourLengthTooHigh,
    WaitForRefresh,
    NoHistory,
}

#[derive(Debug, Clone)]
struct Sector {
    start_angle: i32,
    end_angle: i32,
}

impl Sector {
    fn new(start_angle: i32, end_angle: i32) -> Self {
        Sector {
            start_angle,
            end_angle,
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct ArpaTarget {
    pub(crate) m_status: TargetStatus,

    m_average_contour_length: i32,
    m_small_fast: bool,
    m_previous_contour_length: i32,
    m_lost_count: i32,
    m_refresh_time: u64,
    m_automatic: bool,
    m_radar_pos: GeoPosition,
    m_course: f64,
    m_stationary: i32,
    m_doppler_target: Doppler,
    pub(crate) m_refreshed: RefreshState,
    pub(crate) m_target_id: usize,
    pub(crate) m_transferred_target: bool,
    m_kalman: KalmanFilter,
    pub(crate) contour: Contour,
    m_total_pix: u32,
    m_approaching_pix: u32,
    m_receding_pix: u32,
    have_doppler: bool,
    pub(crate) position: ExtendedPosition,
    pub(crate) expected: Polar,
    pub(crate) age_rotations: u32,
    /// Key of the radar that originally acquired this target
    pub(crate) source_radar: String,
    /// Key of the radar currently tracking this target
    pub(crate) tracking_radar: String,
}

impl HistorySpoke {
    fn new(sweep: Vec<HistoryPixel>, time: u64, pos: GeoPosition) -> Self {
        Self { sweep, time, pos }
    }
}

impl HistorySpokes {
    fn new(stationary: bool, spokes_per_revolution: i32, spoke_len: i32) -> Self {
        log::debug!(
            "creating HistorySpokes ({} x {}) stationary: {}",
            spokes_per_revolution,
            spoke_len,
            stationary
        );
        Self {
            spokes: Box::new(vec![
                HistorySpoke::new(
                    vec![HistoryPixel::new(); 0],
                    0,
                    GeoPosition::new(0., 0.)
                );
                spokes_per_revolution as usize
            ]),
            stationary_layer: if stationary {
                Some(Box::new(Array2::<u8>::zeros((
                    spokes_per_revolution as usize,
                    spoke_len as usize,
                ))))
            } else {
                None
            },
        }
    }

    pub fn mod_spokes(&self, angle: i32) -> usize {
        angle.rem_euclid(self.spokes.len() as i32) as usize
    }

    pub fn pix(&self, doppler: &Doppler, ang: i32, rad: i32) -> bool {
        // Check bounds before casting to usize (negative rad would overflow)
        if self.spokes.is_empty() || rad < 3 || rad as usize >= self.spokes[0].sweep.len() {
            return false;
        }
        let rad = rad as usize;
        let angle = self.mod_spokes(ang);
        if let Some(layer) = &self.stationary_layer {
            if layer[[angle, rad]] != 0 {
                return false;
            }
        }
        let history = self.spokes[angle]
            .sweep
            .get(rad)
            .unwrap_or(&HistoryPixel::INITIAL);
        let target = history.contains(HistoryPixel::TARGET); // above threshold bit
        let backup = history.contains(HistoryPixel::BACKUP); // backup bit does not get cleared when target is refreshed
        let approaching = history.contains(HistoryPixel::APPROACHING); // this is Doppler approaching bit
        let receding = history.contains(HistoryPixel::RECEDING); // this is Doppler receding bit

        match doppler {
            Doppler::Any => target,
            Doppler::NoDoppler => target && !approaching && !receding,
            Doppler::Approaching => approaching,
            Doppler::Receding => receding,
            Doppler::AnyDoppler => approaching || receding,
            Doppler::NotReceding => target && !receding,
            Doppler::NotApproaching => target && !approaching,
            Doppler::AnyPlus => backup,
        }
    }

    fn multi_pix(&mut self, doppler: &Doppler, ang: i32, rad: i32) -> bool {
        // checks if the blob has a contour of at least length pixels
        // pol must start on the contour of the blob
        // false if not
        // if false clears out pixels of the blob in hist

        if !self.pix(doppler, ang, rad) {
            return false;
        }
        let length = MIN_CONTOUR_LENGTH;
        let start = Polar::new(ang as i32, rad as i32, 0);

        let mut current = start; // the 4 possible translations to move from a point on the contour to the next

        let mut max_angle = current;
        let mut min_angle = current;
        let mut max_r = current;
        let mut min_r = current;
        let mut count = 0;
        let mut found = false;

        // first find the orientation of border point p
        let index = {
            let mut index = 0;
            for i in 0..4 {
                if !self.pix(
                    doppler,
                    current.angle + FOUR_DIRECTIONS[i].angle,
                    current.r + FOUR_DIRECTIONS[i].r,
                ) {
                    found = true;
                    break;
                }
                index += 1;
            }
            if !found {
                return false; // single pixel blob
            }
            index
        };
        let mut index = (index + 1) % 4; // determines starting direction

        while current.r != start.r || current.angle != start.angle || count == 0 {
            // Safeguard against infinite loops
            if count > MAX_CONTOUR_LENGTH {
                return false;
            }
            // try all translations to find the next point
            // start with the "left most" translation relative to the
            // previous one
            index = (index + 3) % 4; // we will turn left all the time if possible
            found = false; // Reset found at start of each iteration
            for _ in 0..4 {
                if self.pix(
                    doppler,
                    current.angle + FOUR_DIRECTIONS[index].angle,
                    current.r + FOUR_DIRECTIONS[index].r,
                ) {
                    found = true;
                    break;
                }
                index = (index + 1) % 4;
            }
            if !found {
                return false; // no next point found (this happens when the blob consists of one single pixel)
            } // next point found
            current.angle += FOUR_DIRECTIONS[index].angle;
            current.r += FOUR_DIRECTIONS[index].r;
            if count >= length {
                return true;
            }
            count += 1;
            if current.angle > max_angle.angle {
                max_angle = current;
            }
            if current.angle < min_angle.angle {
                min_angle = current;
            }
            if current.r > max_r.r {
                max_r = current;
            }
            if current.r < min_r.r {
                min_r = current;
            }
        } // contour length is less than m_min_contour_length
        // before returning false erase this blob so we do not have to check this one again
        if min_angle.angle < 0 {
            min_angle.angle += self.spokes.len() as i32;
            max_angle.angle += self.spokes.len() as i32;
        }
        for a in min_angle.angle..=max_angle.angle {
            let a_normalized = self.mod_spokes(a);
            let spoke_sweep_len = self.spokes[a_normalized].sweep.len();
            if spoke_sweep_len == 0 {
                continue;
            }
            for r in min_r.r..=max_r.r {
                if r >= 0 && (r as usize) < spoke_sweep_len {
                    self.spokes[a_normalized].sweep[r as usize] = self.spokes[a_normalized].sweep
                        [r as usize]
                        .intersection(HistoryPixel::NO_TARGET | HistoryPixel::CONTOUR);
                }
            }
        }
        return false;
    }

    // moves pol to contour of blob
    // true if success
    // false when failed
    fn find_contour_from_inside(&mut self, doppler: &Doppler, pol: &mut Polar) -> bool {
        let mut ang = pol.angle;
        let rad = pol.r;
        let mut limit = self.spokes.len() as i32 / 8;

        if !self.pix(doppler, ang, rad) {
            return false;
        }
        while limit >= 0 && self.pix(doppler, ang, rad) {
            ang -= 1;
            limit -= 1;
        }
        ang += 1;
        pol.angle = ang;

        // return true if the blob has the required min contour length
        self.multi_pix(doppler, ang, rad)
    }

    fn pix2(&mut self, doppler: &Doppler, pol: &mut Polar, a: i32, r: i32) -> bool {
        if self.multi_pix(doppler, a, r) {
            pol.angle = a;
            pol.r = r;
            return true;
        }
        return false;
    }

    /// make a search pattern along a square
    /// returns the position of the nearest blob found in pol
    /// dist is search radius (1 more or less) in radial pixels
    fn find_nearest_contour(&mut self, doppler: &Doppler, pol: &mut Polar, dist: i32) -> bool {
        let a = pol.angle;
        let r = pol.r;
        let distance = max(dist, 2);
        let factor: f64 = self.spokes.len() as f64 / 2.0 / PI;

        for j in 1..=distance {
            let dist_r = j;
            let dist_a = max((factor / r as f64 * j as f64) as i32, 1);
            // search starting from the middle
            for i in 0..=dist_a {
                // "upper" side
                if self.pix2(doppler, pol, a - i, r + dist_r) {
                    return true;
                }
                if self.pix2(doppler, pol, a + i, r + dist_r) {
                    return true;
                }
            }
            for i in 0..dist_r {
                // "right hand" side
                if self.pix2(doppler, pol, a + dist_a, r + i) {
                    return true;
                }
                if self.pix2(doppler, pol, a + dist_a, r - i) {
                    return true;
                }
            }
            for i in 0..=dist_a {
                // "lower" side
                if self.pix2(doppler, pol, a - i, r - dist_r) {
                    return true;
                }
                if self.pix2(doppler, pol, a + i, r - dist_r) {
                    return true;
                }
            }
            for i in 0..dist_r {
                // "left hand" side
                if self.pix2(doppler, pol, a - dist_a, r + i) {
                    return true;
                }
                if self.pix2(doppler, pol, a - dist_a, r - i) {
                    return true;
                }
            }
        }
        false
    }

    /**
     * Find a contour from the given start position on the edge of a blob.
     *
     * Follows the contour in a clockwise manner.
     *
     *
     */
    fn get_contour(&mut self, doppler: &Doppler, pol: Polar) -> Result<(Contour, Polar), Error> {
        if self.spokes.is_empty() {
            return Err(Error::NoHistory);
        }

        let mut pol = pol;
        let mut count = 0;
        let mut current = pol;
        let mut next = current;

        let mut succes = false;
        let mut index = 0;

        let mut contour = Contour::new();
        contour.max_r = current.r;
        contour.max_angle = current.angle;
        contour.min_r = current.r;
        contour.min_angle = current.angle;

        // check if p inside blob
        if pol.r as usize >= self.spokes.len() {
            return Err(Error::RangeTooHigh);
        }
        if pol.r < 4 {
            return Err(Error::RangeTooLow);
        }
        if !self.pix(doppler, pol.angle, pol.r) {
            return Err(Error::NoEchoAtStart);
        }

        // first find the orientation of border point p
        for i in 0..4 {
            index = i;
            if !self.pix(
                doppler,
                current.angle + FOUR_DIRECTIONS[index].angle,
                current.r + FOUR_DIRECTIONS[index].r,
            ) {
                succes = true;
                break;
            }
        }
        if !succes {
            return Err(Error::StartPointNotOnContour);
        }
        index = (index + 1) % 4; // determines starting direction

        succes = false;
        while count < MAX_CONTOUR_LENGTH {
            // try all translations to find the next point
            // start with the "left most" translation relative to the previous one
            index = (index + 3) % 4; // we will turn left all the time if possible
            for _i in 0..4 {
                next = current + FOUR_DIRECTIONS[index];
                if self.pix(doppler, next.angle, next.r) {
                    succes = true; // next point found

                    break;
                }
                index = (index + 1) % 4;
            }
            if !succes {
                return Err(Error::BrokenContour);
            }
            // next point found
            current = next;
            if count < MAX_CONTOUR_LENGTH - 1 {
                contour.contour.push(current);
            } else if count == MAX_CONTOUR_LENGTH - 1 {
                contour.contour.push(current);
                contour.contour.push(pol); // shortcut to the beginning for drawing the contour
            }
            if current.angle > contour.max_angle {
                contour.max_angle = current.angle;
            }
            if current.angle < contour.min_angle {
                contour.min_angle = current.angle;
            }
            if current.r > contour.max_r {
                contour.max_r = current.r;
            }
            if current.r < contour.min_r {
                contour.min_r = current.r;
            }
            count += 1;
        }
        contour.length = contour.contour.len() as i32;

        //  CalculateCentroid(*target);    we better use the real centroid instead of the average, TODO

        pol.angle = self.mod_spokes((contour.max_angle + contour.min_angle) / 2) as i32;
        contour.min_angle = self.mod_spokes(contour.min_angle) as i32;
        contour.max_angle = self.mod_spokes(contour.max_angle) as i32;
        pol.r = (contour.max_r + contour.min_r) / 2;
        pol.time = self.spokes[pol.angle as usize].time;

        // TODO        self.m_radar_pos = buffer.history.spokes[pol.angle as usize].pos;

        return Ok((contour, pol));
    }

    fn get_target(
        &mut self,
        doppler: &Doppler,
        pol: Polar,
        dist1: i32,
    ) -> Result<(Contour, Polar), Error> {
        let mut pol = pol;

        // general target refresh

        let dist = min(dist1, pol.r - 5);

        let contour_found = if self.pix(doppler, pol.angle, pol.r) {
            self.find_contour_from_inside(doppler, &mut pol)
        } else {
            self.find_nearest_contour(doppler, &mut pol, dist)
        };
        if !contour_found {
            return Err(Error::NoContourFound);
        }
        self.get_contour(doppler, pol)
    }

    //
    // resets the pixels of the current blob (plus DISTANCE_BETWEEN_TARGETS) so that blob will not be found again in the same sweep
    // We not only reset the blob but all pixels in a radial "square" covering the blob
    fn reset_pixels(&mut self, contour: &Contour, pos: &Polar, pixels_per_meter: &f64) {
        const DISTANCE_BETWEEN_TARGETS: i32 = 30;
        const SHADOW_MARGIN: i32 = 5;
        const TARGET_DISTANCE_FOR_BLANKING_SHADOW: f64 = 6000.; // 6 km

        if self.spokes.is_empty() {
            return;
        }
        let sweep_len = self.spokes[0].sweep.len();
        if sweep_len == 0 {
            return;
        }

        for a in contour.min_angle - DISTANCE_BETWEEN_TARGETS
            ..=contour.max_angle + DISTANCE_BETWEEN_TARGETS
        {
            let a = self.mod_spokes(a);
            let spoke_sweep_len = self.spokes[a].sweep.len();
            if spoke_sweep_len == 0 {
                continue;
            }
            for r in max(contour.min_r - DISTANCE_BETWEEN_TARGETS, 0)
                ..=min(
                    contour.max_r + DISTANCE_BETWEEN_TARGETS,
                    spoke_sweep_len as i32 - 1,
                )
            {
                self.spokes[a].sweep[r as usize] =
                    self.spokes[a].sweep[r as usize].intersection(HistoryPixel::BACKUP);
                // also clear both Doppler bits
            }
        }

        let distance_to_radar = pos.r as f64 / pixels_per_meter;
        // For larger targets clear the "shadow" of the target until 4 * r ????

        if contour.length > 20 && distance_to_radar < TARGET_DISTANCE_FOR_BLANKING_SHADOW {
            let mut max = contour.max_angle;
            if contour.min_angle - SHADOW_MARGIN > contour.max_angle + SHADOW_MARGIN {
                max += self.spokes.len() as i32;
            }
            for a in contour.min_angle - SHADOW_MARGIN..=max + SHADOW_MARGIN {
                let a = self.mod_spokes(a);
                let spoke_sweep_len = self.spokes[a].sweep.len();
                if spoke_sweep_len == 0 {
                    continue;
                }
                for r in contour.max_r as usize..=min(4 * contour.max_r as usize, spoke_sweep_len - 1) {
                    self.spokes[a].sweep[r] =
                        self.spokes[a].sweep[r].intersection(HistoryPixel::BACKUP);
                    // also clear both Doppler bits
                }
            }
        }

        // Draw the contour in the history. This is copied to the output data
        // on the next sweep.
        for p in &contour.contour {
            // Normalize angle (can be negative due to contour tracing) and check r bounds
            let angle = self.mod_spokes(p.angle);
            let spoke_sweep_len = self.spokes[angle].sweep.len();
            if p.r >= 0 && (p.r as usize) < spoke_sweep_len {
                self.spokes[angle].sweep[p.r as usize].insert(HistoryPixel::CONTOUR);
            }
        }
    }
}

impl TargetSetup {
    pub fn polar2pos(&self, pol: &Polar, own_ship: &ExtendedPosition) -> ExtendedPosition {
        // The "own_ship" in the function call can be the position at an earlier time than the current position
        // converts in a radar image angular data r ( 0 - max_spoke_len ) and angle (0 - spokes_per_revolution) to position (lat, lon)
        // based on the own ship position own_ship
        let mut pos: ExtendedPosition = own_ship.clone();
        // should be revised, use Mercator formula PositionBearingDistanceMercator()  TODO
        pos.pos.lat += (pol.r as f64 / self.pixels_per_meter)  // Scale to fraction of distance from radar
                                       * pol.angle_in_rad(self.spokes_per_revolution_f64).cos()
            / METERS_PER_DEGREE_LATITUDE;
        pos.pos.lon += (pol.r as f64 / self.pixels_per_meter)  // Scale to fraction of distance to radar
                                       * pol.angle_in_rad(self.spokes_per_revolution_f64).sin()
            / meters_per_degree_longitude(&own_ship.pos.lat);
        pos
    }

    pub fn pos2polar(&self, p: &ExtendedPosition, own_ship: &ExtendedPosition) -> Polar {
        // converts in a radar image a lat-lon position to angular data relative to position own_ship

        let dif_lat = p.pos.lat - own_ship.pos.lat;
        let dif_lon = (p.pos.lon - own_ship.pos.lon) * own_ship.pos.lat.to_radians().cos();
        let r = ((dif_lat * dif_lat + dif_lon * dif_lon).sqrt()
            * METERS_PER_DEGREE_LATITUDE
            * self.pixels_per_meter
            + 1.) as i32;
        let mut angle =
            f64::atan2(dif_lon, dif_lat) * self.spokes_per_revolution_f64 / (2. * PI) + 1.; // + 1 to minimize rounding errors
        if angle < 0. {
            angle += self.spokes_per_revolution_f64;
        }
        return Polar::new(angle as i32, r, p.time);
    }

    pub fn mod_spokes(&self, angle: i32) -> i32 {
        angle.rem_euclid(self.spokes_per_revolution)
    }

    /// Number of sweeps that a next scan of the target may have moved, 1/10th of circle
    pub fn scan_margin(&self) -> i32 {
        self.spokes_per_revolution / 10
    }
}

impl TargetBuffer {
    pub(crate) fn new(
        stationary: bool,
        info: &RadarInfo,
        shared_manager: Option<manager::SharedTargetManager>,
        sk_client_tx: Option<tokio::sync::broadcast::Sender<crate::stream::SignalKDelta>>,
    ) -> Self {
        let spokes_per_revolution = info.spokes_per_revolution as i32;
        let spoke_len = info.max_spoke_len as i32;
        let radar_key = info.key();

        TargetBuffer {
            setup: TargetSetup {
                key: info.key.clone(),
                spokes_per_revolution,
                spokes_per_revolution_f64: spokes_per_revolution as f64,
                spoke_len,
                have_doppler: info.doppler,
                pixels_per_meter: 0.0,

                rotation_speed_ms: 0,
                stationary,
            },
            next_target_id: 0,
            arpa_via_doppler: false,

            history: HistorySpokes::new(stationary, spokes_per_revolution, spoke_len),
            targets: Arc::new(RwLock::new(HashMap::new())),
            m_clear_contours: false,
            m_auto_learn_state: 0,

            course: 0.,
            course_weight: 0,
            course_samples: 0,

            scanned_angle: -1,
            refreshed_angle: -1,

            shared_manager,
            radar_key,

            guard_zones: [
                DetectionGuardZone::new(spokes_per_revolution),
                DetectionGuardZone::new(spokes_per_revolution),
            ],

            prev_angle: 0,

            current_heading: 0,

            blobs_in_progress: Vec::new(),

            sk_client_tx,
        }
    }

    /// Broadcast a target update to connected SignalK clients
    fn broadcast_target_update(&self, target: &ArpaTarget, radar_position: &GeoPosition) {
        // Don't broadcast early acquiring states
        if !target.should_broadcast() {
            return;
        }
        if let Some(ref tx) = self.sk_client_tx {
            let target_api = target.to_api(radar_position);
            let mut delta = crate::stream::SignalKDelta::new();
            delta.add_target_update(&self.radar_key, target.m_target_id, Some(target_api));
            // Ignore send errors - no receivers is normal when no clients connected
            let _ = tx.send(delta);
        }
    }

    /// Broadcast that a target was lost
    fn broadcast_target_lost(&self, target_id: usize) {
        if let Some(ref tx) = self.sk_client_tx {
            let mut delta = crate::stream::SignalKDelta::new();
            delta.add_target_update(&self.radar_key, target_id, None);
            let _ = tx.send(delta);
        }
    }

    /// Update guard zone configuration from settings
    pub fn set_guard_zone(
        &mut self,
        zone_index: usize,
        start_angle_rad: f64,
        end_angle_rad: f64,
        inner_range_m: f64,
        outer_range_m: f64,
        enabled: bool,
    ) {
        if zone_index < 2 {
            self.guard_zones[zone_index].update_from_config(
                start_angle_rad,
                end_angle_rad,
                inner_range_m,
                outer_range_m,
                enabled,
                self.setup.spokes_per_revolution,
                self.setup.pixels_per_meter,
            );
        }
    }

    /// Check if any guard zone is enabled
    fn has_active_guard_zone(&self) -> bool {
        self.guard_zones.iter().any(|z| z.enabled)
    }

    /// Check if a position is within any enabled guard zone
    fn is_in_guard_zone(&self, angle: i32, range: i32, heading: i32) -> bool {
        self.guard_zones
            .iter()
            .any(|z| z.contains(angle, range, self.setup.spokes_per_revolution, heading))
    }

    pub fn set_rotation_speed(&mut self, ms: u32) {
        self.setup.rotation_speed_ms = ms;
    }

    pub fn set_arpa_via_doppler(&mut self, arpa: bool) -> Result<(), ControlError> {
        if arpa && !self.setup.have_doppler {
            return Err(ControlError::NotSupported(ControlId::DopplerAutoTrack));
        }
        self.arpa_via_doppler = arpa;
        Ok(())
    }

    fn reset_history(&mut self) {
        self.history = HistorySpokes::new(
            self.setup.stationary,
            self.setup.spokes_per_revolution,
            self.setup.spoke_len,
        );
        // Clear any blobs in progress since history was reset
        self.blobs_in_progress.clear();
    }

    fn clear_contours(&mut self) {
        for (_, t) in self.targets.write().unwrap().iter_mut() {
            t.contour.length = 0;
            t.m_average_contour_length = 0;
        }
    }
    fn get_next_target_id(&mut self) -> usize {
        const MAX_TARGET_ID: usize = 100000;

        self.next_target_id += 1;
        if self.next_target_id >= MAX_TARGET_ID {
            self.next_target_id = 1;
        }

        self.next_target_id
    }

    //
    // FUNCTIONS COMING FROM "RadarArpa"
    //

    fn find_target_id_by_position(&self, pos: &GeoPosition) -> Option<usize> {
        let mut best_id = None;
        let mut min_dist = 1000.;
        for (id, target) in self.targets.read().unwrap().iter() {
            if target.m_status != TargetStatus::Lost {
                let dif_lat = pos.lat - target.position.pos.lat;
                let dif_lon = (pos.lon - target.position.pos.lon) * pos.lat.to_radians().cos();
                let dist2 = dif_lat * dif_lat + dif_lon * dif_lon;
                if dist2 < min_dist {
                    min_dist = dist2;
                    best_id = Some(*id);
                }
            }
        }

        best_id
    }

    fn acquire_new_marpa_target(&mut self, target_pos: ExtendedPosition) {
        self.acquire_or_delete_marpa_target(target_pos, TargetStatus::Acquire0);
    }

    /// Delete the target that is closest to the position
    fn delete_target(&mut self, pos: &GeoPosition) {
        // In dual radar mode, use shared manager
        if let Some(ref manager) = self.shared_manager {
            if manager.delete_target_near(pos).is_none() {
                log::debug!(
                    "Could not find (M)ARPA target to delete within 1000 meters from {}",
                    pos
                );
            }
        } else {
            // Single radar mode - use local storage
            if let Some(id) = self.find_target_id_by_position(pos) {
                self.targets.write().unwrap().remove(&id);
            } else {
                log::debug!(
                    "Could not find (M)ARPA target to delete within 1000 meters from {}",
                    pos
                );
            }
        }
    }

    fn acquire_or_delete_marpa_target(
        &mut self,
        target_pos: ExtendedPosition,
        status: TargetStatus,
    ) {
        // acquires new target from mouse click position
        // no contour taken yet
        // target status acquire0
        // returns in X metric coordinates of click
        // constructs Kalman filter
        // make new target

        log::debug!("Adding (M)ARPA target at {}", target_pos.pos);

        // In dual radar mode, use shared manager for target storage
        if let Some(ref manager) = self.shared_manager {
            manager.acquire_target(
                &self.radar_key,
                target_pos,
                Doppler::Any,
                self.setup.spokes_per_revolution as usize,
                self.setup.have_doppler,
            );
        } else {
            // Single radar mode - use local storage
            let id = self.get_next_target_id();
            let mut target = ArpaTarget::new(
                target_pos,
                GeoPosition::new(0., 0.),
                id,
                self.setup.spokes_per_revolution as usize,
                status,
                self.setup.have_doppler,
            );
            target.source_radar = self.radar_key.clone();
            target.tracking_radar = self.radar_key.clone();
            self.targets.write().unwrap().insert(id, target);
        }
    }

    fn cleanup_lost_targets(&mut self) {
        // In dual radar mode, cleanup via shared manager
        if let Some(ref manager) = self.shared_manager {
            manager.cleanup_lost_targets();
        }

        // Also cleanup local targets
        self.targets
            .write()
            .unwrap()
            .retain(|_, t| t.m_status != TargetStatus::Lost);
        for (_, v) in self.targets.write().unwrap().iter_mut() {
            v.m_refreshed = RefreshState::NotFound;
        }
    }

    ///
    /// Refresh all targets between two angles
    ///
    fn refresh_all_arpa_targets(&mut self, start_angle: i32, end_angle: i32) {
        if self.setup.pixels_per_meter == 0. {
            return;
        }

        let target_count = self.targets.read().unwrap().len();
        if target_count > 0 {
            // Log target angles for each segment refresh
            let target_angles: Vec<i32> = self
                .targets
                .read()
                .unwrap()
                .values()
                .map(|t| t.contour.position.angle)
                .collect();
            log::debug!(
                "refresh_all_arpa_targets: {} targets at angles {:?}, scanning range {}..{}",
                target_count,
                target_angles,
                start_angle,
                end_angle
            );
        }

        self.cleanup_lost_targets();

        // main target refresh loop

        // pass 0 of target refresh  Only search for moving targets faster than 2 knots as long as autolearnng is initializing
        // When autolearn is ready, apply for all targets

        let speed = MAX_DETECTION_SPEED_KN * KN_TO_MS; // m/sec
        let search_radius =
            (speed * TODO_ROTATION_SPEED_MS as f64 * self.setup.pixels_per_meter / 1000.) as i32;

        // In dual radar mode, get targets assigned to this radar from shared manager
        if let Some(ref manager) = self.shared_manager {
            let targets_for_radar = manager.get_targets_for_radar(&self.radar_key);
            log::debug!(
                "refresh_all_arpa_targets: shared manager mode, {} targets for radar {}",
                targets_for_radar.len(),
                self.radar_key
            );
            for (target_id, target) in targets_for_radar {
                self.refresh_single_target(
                    target_id,
                    target,
                    start_angle,
                    end_angle,
                    search_radius,
                );
            }
        } else {
            // Single radar mode - use local storage
            let target_ids: Vec<usize> = self.targets.read().unwrap().keys().cloned().collect();
            log::debug!(
                "refresh_all_arpa_targets: single radar mode, {} target_ids: {:?}",
                target_ids.len(),
                target_ids
            );
            for target_id in target_ids {
                let target = {
                    let targets = self.targets.read().unwrap();
                    match targets.get(&target_id) {
                        Some(t) => t.clone(),
                        None => {
                            log::debug!("Target {} not found in storage", target_id);
                            continue;
                        }
                    }
                };
                log::debug!(
                    "Processing local target {} at angle {}",
                    target_id,
                    target.contour.position.angle
                );
                self.refresh_single_target(
                    target_id,
                    target,
                    start_angle,
                    end_angle,
                    search_radius,
                );
            }
        }
    }

    fn refresh_single_target(
        &mut self,
        target_id: usize,
        mut target: ArpaTarget,
        start_angle: i32,
        end_angle: i32,
        search_radius: i32,
    ) {
        if !target
            .contour
            .position
            .angle_is_between(start_angle, end_angle)
        {
            return;
        }

        log::info!(
            "Refreshing target {}: status={:?}, angle={}, range={}..{}",
            target_id,
            target.m_status,
            target.contour.position.angle,
            start_angle,
            end_angle
        );

        for pass in Pass::iter() {
            let radius = match pass {
                Pass::First => search_radius / 4,
                Pass::Second => search_radius / 3,
                Pass::Third => search_radius,
            };

            if pass == Pass::First
                && !((target.position.speed_kn >= 2.5
                    && target.age_rotations >= TODO_TARGET_AGE_TO_MIXER)
                    || self.m_auto_learn_state >= 1)
            {
                continue;
            }

            let prev_status = target.m_status.clone();
            let clone = target.clone();
            match ArpaTarget::refresh_target(
                clone,
                &self.setup,
                &mut self.history,
                radius / 4,
                pass,
            ) {
                Ok(t) => {
                    // Log status changes
                    if t.m_status != prev_status {
                        log::info!(
                            "Target {} status: {:?} -> {:?}, speed={:.1}kn, lost_count={}",
                            t.m_target_id,
                            prev_status,
                            t.m_status,
                            t.position.speed_kn,
                            t.m_lost_count
                        );
                    }
                    target = t;
                }
                Err(e) => match e {
                    Error::Lost => {
                        log::info!("Target {} lost", target.m_target_id);
                        target.m_status = TargetStatus::Lost;
                    }
                    _ => {
                        log::debug!("Target {} refresh error {:?}", target.m_target_id, e);
                    }
                },
            }
        }

        // Broadcast target state to SignalK clients
        if target.m_status == TargetStatus::Lost {
            self.broadcast_target_lost(target_id);
        } else {
            self.broadcast_target_update(&target, &target.m_radar_pos);
        }

        // Update the target back to storage
        if let Some(ref manager) = self.shared_manager {
            manager.update_target(target_id, target, self.history.spokes[0].time);
        } else {
            self.targets.write().unwrap().insert(target_id, target);
        }
    }

    pub fn delete_all_targets(&mut self) {
        // Broadcast lost for all targets before deleting
        let target_ids: Vec<usize> = if let Some(ref manager) = self.shared_manager {
            manager.get_all_target_ids()
        } else {
            self.targets.read().unwrap().keys().copied().collect()
        };
        for target_id in target_ids {
            self.broadcast_target_lost(target_id);
        }

        // In dual radar mode, use shared manager
        if let Some(ref manager) = self.shared_manager {
            manager.delete_all_targets();
        }
        // Always clear local storage too
        self.targets.write().unwrap().clear();
    }

    /**
     * Inject the target in this radar.
     *
     * Called from the main thread and from the InterRadar receive thread.
     */
    /*
    void RadarArpa::InsertOrUpdateTargetFromOtherRadar(const DynamicTargetData* data, bool remote) {
      wxCriticalSectionLocker lock(m_ri.m_exclusive);

      // This method works on the other radar than TransferTargetToOtherRadar
      // find target
      bool found = false;
      int uid = data->target_id;
      LOG_ARPA(wxT("%s: InsertOrUpdateTarget id=%i"), m_ri.m_name, uid);
      ArpaTarget* updated_target = 0;
      for (auto target = m_targets.begin(); target != m_targets.end(); target++) {
        //LOG_ARPA(wxT("%s: InsertOrUpdateTarget id=%i, found=%i"), m_ri.m_name, uid, (*target).m_target_id);
        if ((*target).m_target_id == uid) {  // target found!
          updated_target = (*target).get();
          found = true;
          LOG_ARPA(wxT("%s: InsertOrUpdateTarget found target id=%d pos=%d"), m_ri.m_name, uid, target - m_targets.begin());
          break;
        }
      }
      if (!found) {
        // make new target with existing uid
        LOG_ARPA(wxT("%s: InsertOrUpdateTarget new target id=%d, pos=%ld"), m_ri.m_name, uid, m_targets.size());
    #ifdef __WXMSW__
        std::unique_ptr<ArpaTarget> new_target = std::make_unique<ArpaTarget>(m_pi, m_ri, uid);
        #else
        std::unique_ptr<ArpaTarget> new_target = make_unique<ArpaTarget>(m_pi, m_ri, uid);
        #endif
        updated_target = new_target.get();
        m_targets.push_back(std::move(new_target));
        ExtendedPosition own_pos;
        if (remote) {
          m_ri->GetRadarPosition(&own_pos);
          Polar pol = updated_target->Pos2Polar(data->position, own_pos);
          LOG_ARPA(wxT("%s: InsertOrUpdateTarget new target id=%d polar=%i"), m_ri.m_name, uid, pol.angle);
          // set estimated time of last refresh as if it was a local target
          updated_target.m_refresh_time = m_ri.m_history[MOD_SPOKES(pol.angle)].time;
        }
      }
      //LOG_ARPA(wxT("%s: InsertOrUpdateTarget processing id=%i"), m_ri.m_name, uid);
      updated_target.m_kalman.P = data->P;
      updated_target.m_position = data->position;
      updated_target.m_status = data->status;
      LOG_ARPA(wxT("%s: transferred id=%i, lat= %f, lon= %f, status=%i,"), m_ri.m_name, updated_target.m_target_id,
               updated_target.m_position.pos.lat, updated_target.m_position.pos.lon, updated_target.m_status);
      updated_target.m_doppler_target = ANY;
      updated_target.m_lost_count = 0;
      updated_target.m_automatic = true;
      double s1 = updated_target.m_position.dlat_dt;  // m per second
      double s2 = updated_target.m_position.dlon_dt;                                   // m  per second
      updated_target.m_course = rad2deg(atan2(s2, s1));
      if (remote) {   // inserted or updated target originated from another radar
        updated_target.m_transferred_target = true;
        //LOG_ARPA(wxT(" m_transferred_target = true targetid=%i"), updated_target.m_target_id);
      }
      return;
    }
    */
    /*

    bool RadarArpa::AcquireNewARPATarget(Polar pol, int status, Doppler doppler) {
      // acquires new target at polar position pol
      // no contour taken yet
      // target status status, normally 0, if dummy target to delete a target -2
      // constructs Kalman filter
      ExtendedPosition own_pos;
      ExtendedPosition target_pos;
      Doppler doppl = doppler;
      if (!m_ri->GetRadarPosition(&own_pos.pos)) {
        return false;
      }

      // make new target
    #ifdef __WXMSW__
      std::unique_ptr<ArpaTarget> target = std::make_unique<ArpaTarget>(m_pi, m_ri, 0);
      #else
      std::unique_ptr<ArpaTarget> target = make_unique<ArpaTarget>(m_pi, m_ri, 0);
      #endif
      target_pos = target->Polar2Pos(pol, own_pos);
      target.m_doppler_target = doppl;
      target.m_position = target_pos;  // Expected position
      target.m_position.time = wxGetUTCTimeMillis();
      target.m_position.dlat_dt = 0.;
      target.m_position.dlon_dt = 0.;
      target.m_position.speed_kn = 0.;
      target.m_position.sd_speed_kn = 0.;
      target.m_status = status;
      target.m_max_angle.angle = 0;
      target.m_min_angle.angle = 0;
      target.m_max_r.r = 0;
      target.m_min_r.r = 0;
      target.m_doppler_target = doppl;
      target.m_refreshed = NOT_FOUND;
      target.m_automatic = true;
      target->RefreshTarget(TARGET_SEARCH_RADIUS1, 1);

      m_targets.push_back(std::move(target));
      return true;
    }

    void RadarArpa::ClearContours() { m_clear_contours = true; }
    */

    /*
    void RadarArpa::ProcessIncomingMessages() {
      DynamicTargetData* target;
      if (m_clear_contours) {
        m_clear_contours = false;
        for (auto target = m_targets.begin(); target != m_targets.end(); target++) {
          (*target).m_contour_length = 0;
          (*target).m_previous_contour_length = 0;
        }
      }

      while ((target = GetIncomingRemoteTarget()) != NULL) {
        InsertOrUpdateTargetFromOtherRadar(target, true);
        delete target;
      }
    }
      */

    /*
    bool RadarArpa::IsAtLeastOneRadarTransmitting() {
      for (size_t r = 0; r < RADARS; r++) {
        if (m_pi.m_radar[r] != NULL && m_pi.m_radar[r].m_state.GetValue() == RADAR_TRANSMIT) {
          return true;
        }
      }
      return false;
    }
      */

    /*
    void RadarArpa::SearchDopplerTargets() {
      ExtendedPosition own_pos;

      if (!m_pi.m_settings.show                       // No radar shown
          || !m_ri->GetRadarPosition(&own_pos.pos)     // No position
          || m_pi->GetHeadingSource() == HEADING_NONE  // No heading
          || (m_pi->GetHeadingSource() == HEADING_FIX_HDM && m_pi.m_var_source == VARIATION_SOURCE_NONE)) {
        return;
      }

      if (m_ri.m_pixels_per_meter == 0. || !IsAtLeastOneRadarTransmitting()) {
        return;
      }

      size_t range_start = 20;  // Convert from meters to 0..511
      size_t range_end;
      int outer_limit = m_ri.m_spoke_len_max;
      outer_limit = (int)outer_limit * 0.93;
      range_end = outer_limit;  // Convert from meters to 0..511

      SpokeBearing start_bearing = 0;
      SpokeBearing end_bearing = m_ri.m_spokes;

      // loop with +2 increments as target must be larger than 2 pixels in width
      for (int angleIter = start_bearing; angleIter < end_bearing; angleIter += 2) {
        SpokeBearing angle = MOD_SPOKES(angleIter);
        wxLongLong angle_time = m_ri.m_history[angle].time;
        // angle_time_plus_margin must be timed later than the pass 2 in refresh, otherwise target may be found multiple times
        wxLongLong angle_time_plus_margin = m_ri.m_history[MOD_SPOKES(angle + 3 * SCAN_MARGIN)].time;

        // check if target has been refreshed since last time
        // and if the beam has passed the target location with SCAN_MARGIN spokes
        if ((angle_time > (m_doppler_arpa_update_time[angle] + SCAN_MARGIN2) &&
             angle_time_plus_margin >= angle_time)) {  // the beam sould have passed our "angle" AND a
                                                       // point SCANMARGIN further set new refresh time
          m_doppler_arpa_update_time[angle] = angle_time;
          for (int rrr = (int)range_start; rrr < (int)range_end; rrr++) {
            if (m_ri.m_arpa->MultiPix(angle, rrr, ANY_DOPPLER)) {
              // pixel found that does not belong to a known target
              Polar pol;
              pol.angle = angle;
              pol.r = rrr;
              if (!m_ri.m_arpa->AcquireNewARPATarget(pol, 0, ANY_DOPPLER)) {
                break;
              }
            }
          }
        }
      }

      return;
    }
    */

    /*
    DynamicTargetData* RadarArpa::GetIncomingRemoteTarget() {
      wxCriticalSectionLocker lock(m_remote_target_lock);
      DynamicTargetData* next;
      if (m_remote_target_queue.empty()) {
        next = NULL;
      } else {
        next = m_remote_target_queue.front();
        m_remote_target_queue.pop_front();
      }
      return next;
    }
    */

    /*
     * Safe to call from any thread
     */
    /*
    void RadarArpa::StoreRemoteTarget(DynamicTargetData* target) {
      wxCriticalSectionLocker lock(m_remote_target_lock);
      m_remote_target_queue.push_back(target);
    }
      */

    fn sample_course(&mut self, bearing: &Option<u32>) {
        let hdt = bearing
            .map(|x| x as f64 / self.setup.spokes_per_revolution_f64)
            .or_else(|| navdata::get_heading_true());

        if let Some(mut hdt) = hdt {
            self.course_samples += 1;
            if self.course_samples == 128 {
                self.course_samples = 0;
                while self.course - hdt > 180. {
                    hdt += 360.;
                }
                while self.course - hdt < -180. {
                    hdt -= 360.;
                }
                if self.course_weight < 16 {
                    self.course_weight += 1;
                }
                self.course += (self.course - hdt) / self.course_weight as f64;
            }
        }
    }

    fn acquire_new_arpa_target(
        &mut self,
        pol: Polar,
        own_pos: GeoPosition,
        time: u64,
        status: TargetStatus,
        doppler: &Doppler,
        automatic: bool,
    ) {
        let epos = ExtendedPosition::new(own_pos.clone(), 0., 0., time, 0., 0.);
        let target_pos = self.setup.polar2pos(&pol, &epos);
        let uid = self.get_next_target_id();

        let mut target = ArpaTarget::new(
            target_pos.clone(),
            own_pos,
            uid,
            self.setup.spokes_per_revolution as usize,
            status,
            *doppler == Doppler::AnyDoppler,
        );

        // Set the contour position so refresh_targets can find this target
        target.contour.position = pol;
        target.expected = pol;
        target.m_automatic = automatic;

        log::info!(
            "Target {} acquired: status={:?}, angle={}, range={}, lat={:.6}, lon={:.6}",
            uid,
            target.m_status,
            pol.angle,
            pol.r,
            target_pos.pos.lat,
            target_pos.pos.lon
        );

        // Broadcast new target to SignalK clients
        self.broadcast_target_update(&target, &target.m_radar_pos);

        // Store to shared manager in dual radar mode, otherwise local storage
        if let Some(ref manager) = self.shared_manager {
            manager.add_target(uid, target, &self.radar_key);
        } else {
            self.targets.write().unwrap().insert(uid, target);
        }
    }

    /// Work on the targets when spoke `angle` has just been processed.
    /// We look for targets a while back, so one quarter rotation ago.
    /// Get the number of targets currently being tracked
    fn target_count(&self) -> usize {
        if let Some(ref manager) = self.shared_manager {
            manager.target_count()
        } else {
            self.targets.read().unwrap().len()
        }
    }

    /// Work on the targets when spoke `angle` has just been processed.
    /// Refresh targets in 1/32th segments of the revolution, looking at the segment
    /// that is 25-50% ahead of the current angle.
    fn refresh_targets(&mut self, angle: usize) {
        // Segment size: 1/32th of revolution
        let segment_size = self.setup.spokes_per_revolution / 32;

        // Calculate which segment we should be refreshing (25-50% ahead)
        let refresh_offset = REFRESH_START_PERCENTAGE * self.setup.spokes_per_revolution / 100;
        let target_angle = self.setup.mod_spokes(angle as i32 + refresh_offset);

        // Which segment does this angle fall into?
        let current_segment = target_angle / segment_size;

        // Initialize refreshed_angle to track which segment we last processed
        if self.refreshed_angle == -1 {
            self.refreshed_angle = current_segment;
        }

        // Only process if we've moved to a new segment
        if current_segment != self.refreshed_angle {
            let start_angle = self.setup.mod_spokes(current_segment * segment_size);
            let end_angle = self.setup.mod_spokes((current_segment + 1) * segment_size);

            self.refresh_all_arpa_targets(start_angle, end_angle);
            self.refreshed_angle = current_segment;
        }
    }

    pub(crate) fn process_spoke(&mut self, spoke: &mut Spoke, legend: &Legend) {
        if spoke.range == 0 {
            return;
        }

        let pos = if let (Some(lat), Some(lon)) = (spoke.lat, spoke.lon) {
            GeoPosition { lat, lon }
        } else {
            log::info!("No radar pos, no (M)ARPA possible");
            return;
        };

        let time = spoke.time.unwrap();
        self.sample_course(&spoke.bearing); // Calculate course as the moving average of m_hdt over one revolution

        // TODO main bang size erase
        // TODO: Range Adjustment compensation

        let pixels_per_meter = spoke.data.len() as f64 / spoke.range as f64;

        if self.setup.pixels_per_meter != pixels_per_meter {
            log::debug!(
                " detected spoke range change from {} to {} pixels/m, {} meters",
                self.setup.pixels_per_meter,
                pixels_per_meter,
                spoke.range
            );
            let old_ppm = self.setup.pixels_per_meter;
            self.setup.pixels_per_meter = pixels_per_meter;
            self.reset_history();
            self.clear_contours();

            // Recalculate guard zones when pixels_per_meter changes
            for zone in &mut self.guard_zones {
                if zone.has_pending_config() || zone.enabled {
                    zone.recalculate(self.setup.spokes_per_revolution, pixels_per_meter);
                }
            }
        }

        // For ARPA to work correctly, history must always be stored using geographic bearing
        // (like the C++ code does with m_history[bearing]). pos2polar calculates geographic
        // bearing, so the history index must also be geographic for lookups to work.
        // When no bearing is available (no heading source), we fall back to relative angle
        // but ARPA tracking will not work correctly in this case.
        let weakest_normal_blob = legend.strong_return;
        let spokes = self.setup.spokes_per_revolution;
        let (angle, heading) = if let Some(bearing) = spoke.bearing {
            // Geographic bearing available - calculate heading as difference
            let heading = (bearing as i32 - spoke.angle as i32).rem_euclid(spokes);
            (bearing as usize, heading)
        } else {
            // No heading available - fall back to relative angle (ARPA won't work correctly)
            (spoke.angle as usize, 0)
        };

        // Store heading for use by process_completed_blob
        self.current_heading = heading;

        let background_on = self.setup.stationary; // TODO m_autolearning_on_off.GetValue() == 1;

        // Overlay blob edges from previous rotation BEFORE clearing history
        self.overlay_blob_edges(spoke, angle, legend);

        self.history.spokes[angle].time = time;
        self.history.spokes[angle].sweep.clear();
        self.history.spokes[angle].pos = pos;
        self.history.spokes[angle]
            .sweep
            .resize(spoke.data.len(), HistoryPixel::empty());

        // Collect strong return pixels from this spoke
        let mut strong_pixels: Vec<i32> = Vec::new();
        for radius in 0..spoke.data.len() {
            if spoke.data[radius] >= weakest_normal_blob {
                self.history.spokes[angle].sweep[radius] = HistoryPixel::INITIAL;
                if background_on {
                    if let Some(layer) = self.history.stationary_layer.as_deref_mut() {
                        if layer[[angle, radius]] < u8::MAX {
                            layer[[angle, radius]] += 1;
                        }
                    }
                }
                // Only consider pixels beyond main bang for blob detection
                if radius as i32 >= MIN_BLOB_RANGE {
                    strong_pixels.push(radius as i32);
                }
            }

            if Some(spoke.data[radius]) == legend.doppler_approaching {
                self.history.spokes[angle].sweep[radius].insert(HistoryPixel::APPROACHING);
            }

            if Some(spoke.data[radius]) == legend.doppler_receding {
                self.history.spokes[angle].sweep[radius].insert(HistoryPixel::RECEDING);
            }
        }

        // Build blobs incrementally
        self.process_blob_pixels(angle as i32, heading, &strong_pixels, time, pos.clone());

        self.refresh_targets(angle);
    }

    /// Overlay blob edge markers from history onto the spoke data
    fn overlay_blob_edges(&self, spoke: &mut Spoke, angle: usize, legend: &Legend) {
        if angle >= self.history.spokes.len() {
            return;
        }

        let sweep = &self.history.spokes[angle].sweep;
        let blob_edge_color = legend.target_border;

        for (radius, pixel) in sweep.iter().enumerate() {
            if pixel.contains(HistoryPixel::BLOB_EDGE) && radius < spoke.data.len() {
                spoke.data[radius] = blob_edge_color;
            }
        }
    }

    /// Process strong pixels from a spoke and update blobs in progress
    fn process_blob_pixels(
        &mut self,
        angle: i32,
        heading: i32,
        strong_pixels: &[i32],
        time: u64,
        pos: GeoPosition,
    ) {
        let spokes = self.setup.spokes_per_revolution;

        // Check if any guard zone is enabled - automatic target acquisition ONLY happens
        // within enabled guard zones (matching C++ radar_pi behavior)
        let guard_zone_active = self.guard_zones[0].enabled || self.guard_zones[1].enabled;

        // Log guard zone state once per rotation (at angle 0)
        if angle == 0 {
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
                self.complete_all_blobs();
            }
            self.prev_angle = angle as usize;
            return;
        }

        // Handle angle wraparound - complete all blobs when we wrap
        if !self.blobs_in_progress.is_empty() {
            let first_blob_angle = self.blobs_in_progress[0].min_angle;
            // If we've wrapped around and are back near where blobs started, complete them all
            if angle < first_blob_angle && self.prev_angle as i32 > angle {
                self.complete_all_blobs();
            }
        }

        // Filter pixels by guard zone and group into contiguous runs
        // A run is a sequence of pixels where each is adjacent (r differs by 1)
        let mut runs: Vec<Vec<i32>> = Vec::new();
        let mut current_run: Vec<i32> = Vec::new();

        for &r in strong_pixels {
            // Check if pixel is in a guard zone
            let in_zone = self.guard_zones[0].contains(angle, r, spokes, heading)
                || self.guard_zones[1].contains(angle, r, spokes, heading);
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
        let prev_angle = (angle - 1).rem_euclid(spokes);

        // For each run, find ALL adjacent blobs (there may be multiple that need merging)
        for (run_idx, run) in runs.iter().enumerate() {
            let mut adjacent_blob_indices: Vec<usize> = Vec::new();

            for (blob_idx, blob) in self.blobs_in_progress.iter().enumerate() {
                // Only consider blobs whose last_angle is the previous spoke
                if blob.last_angle != prev_angle {
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
                // Merge the blob data
                primary.min_angle = min(primary.min_angle, merge_blob.min_angle);
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
            blob.start_new_spoke(angle);
            for &r in run {
                blob.add_pixel(angle, r);
                blob.last_spoke_ranges.push(r);
            }
        }

        // Start new blobs for unassigned runs
        for (run_idx, run) in runs.iter().enumerate() {
            if run_assigned[run_idx] {
                continue;
            }
            // Create a new blob with all pixels in this run
            let mut blob = BlobInProgress::new(angle, run[0], time, pos.clone());
            for &r in run.iter().skip(1) {
                blob.add_pixel(angle, r);
                blob.last_spoke_ranges.push(r);
            }
            self.blobs_in_progress.push(blob);
        }

        // Find completed blobs: those that weren't extended this spoke
        // AND whose last_angle is before the previous spoke (so they had a gap)
        let mut completed_indices: Vec<usize> = Vec::new();
        for (idx, blob) in self.blobs_in_progress.iter().enumerate() {
            // Blob is complete if it wasn't extended and last_angle < prev_angle
            // (meaning there's been at least one spoke with no contribution)
            if blob.last_angle != angle && blob.last_angle != prev_angle {
                completed_indices.push(idx);
            }
        }

        // Process completed blobs (in reverse order to preserve indices during removal)
        for &idx in completed_indices.iter().rev() {
            let blob = self.blobs_in_progress.remove(idx);
            self.process_completed_blob(blob);
        }
    }

    /// Complete all blobs in progress (called on angle wraparound or range change)
    fn complete_all_blobs(&mut self) {
        let blobs: Vec<BlobInProgress> = self.blobs_in_progress.drain(..).collect();
        for blob in blobs {
            self.process_completed_blob(blob);
        }
    }

    /// Process a completed blob - check validity and pass to target acquisition
    fn process_completed_blob(&mut self, blob: BlobInProgress) {
        if !blob.is_valid() {
            log::trace!(
                "Blob rejected: pixels={}, range={}..{}, angles={}..{}",
                blob.pixel_count,
                blob.min_r,
                blob.max_r,
                blob.min_angle,
                blob.max_angle
            );
            return;
        }

        let (center_angle, center_r) = blob.center();
        let spokes = self.setup.spokes_per_revolution;
        let heading = self.current_heading;

        // Verify blob is within guard zone (if enabled)
        let guard_zone_active = self.guard_zones[0].enabled || self.guard_zones[1].enabled;
        if guard_zone_active {
            let in_zone0 = self.guard_zones[0].contains(center_angle, center_r, spokes, heading);
            let in_zone1 = self.guard_zones[1].contains(center_angle, center_r, spokes, heading);
            if !in_zone0 && !in_zone1 {
                log::warn!(
                    "Blob outside guard zones! center=({}, {}), zone0={}..{}/{}..{}, zone1={}..{}/{}..{}",
                    center_angle,
                    center_r,
                    self.guard_zones[0].start_angle,
                    self.guard_zones[0].end_angle,
                    self.guard_zones[0].inner_range,
                    self.guard_zones[0].outer_range,
                    self.guard_zones[1].start_angle,
                    self.guard_zones[1].end_angle,
                    self.guard_zones[1].inner_range,
                    self.guard_zones[1].outer_range
                );
                return;
            }
        }

        // Get the ship position at the center angle from history
        // This is more accurate than blob.start_pos which is from when the blob started
        let center_angle_idx = (center_angle as usize) % self.history.spokes.len();
        let center_pos = self.history.spokes[center_angle_idx].pos.clone();
        let center_time = self.history.spokes[center_angle_idx].time;

        log::info!(
            "Blob detected: {} pixels, center=({}, {}), range={}..{}, angles={}..{}, pos=({:.6}, {:.6})",
            blob.pixel_count,
            center_angle,
            center_r,
            blob.min_r,
            blob.max_r,
            blob.min_angle,
            blob.max_angle,
            center_pos.lat,
            center_pos.lon
        );

        // Mark blob edges in history for overlay visualization
        self.mark_blob_edges(&blob);

        // Create a Polar position for the blob center
        let pol = Polar::new(center_angle, center_r, center_time);

        // Pass to target acquisition with the ship position at the center angle
        self.acquire_or_match_blob(pol, center_pos);
    }

    /// Mark the entire blob area in the history array for visualization overlay
    fn mark_blob_edges(&mut self, blob: &BlobInProgress) {
        let spokes = self.setup.spokes_per_revolution as usize;
        log::debug!(
            "mark_blob_edges: blob angle=[{}..{}] r=[{}..{}]",
            blob.min_angle,
            blob.max_angle,
            blob.min_r,
            blob.max_r
        );

        // Fill the entire blob bounding box
        for angle in blob.min_angle..=blob.max_angle {
            let a = (angle as usize) % spokes;
            if a >= self.history.spokes.len() {
                continue;
            }
            let sweep_len = self.history.spokes[a].sweep.len();

            // Mark all pixels from min_r to max_r
            for r in blob.min_r..=blob.max_r {
                let r_idx = r as usize;
                if r_idx < sweep_len {
                    self.history.spokes[a].sweep[r_idx].insert(HistoryPixel::BLOB_EDGE);
                }
            }
        }
    }

    /// Try to match blob to existing target, or acquire as new target
    fn acquire_or_match_blob(&mut self, pol: Polar, center_pos: GeoPosition) {
        // Search radius for matching blobs to targets (in pixels)
        // This should be larger than typical target movement between sweeps
        const MATCH_RADIUS: i32 = 20;

        // Check if blob matches any existing target's expected position
        let targets = self.targets.read().unwrap();
        let spokes = self.setup.spokes_per_revolution;

        for (target_id, target) in targets.iter() {
            if target.m_status == TargetStatus::Lost {
                continue;
            }

            // Calculate distance from blob center to target's expected position
            let angle_diff = (pol.angle - target.expected.angle).abs();
            let angle_diff = angle_diff.min(spokes - angle_diff); // Handle wrap-around
            let range_diff = (pol.r - target.expected.r).abs();

            // Check if within search radius (use squared distance for efficiency)
            let dist_sq = angle_diff * angle_diff + range_diff * range_diff;
            if dist_sq <= MATCH_RADIUS * MATCH_RADIUS {
                log::debug!(
                    "Blob matches existing target {}: blob=({},{}), expected=({},{}), dist={}",
                    target_id,
                    pol.angle,
                    pol.r,
                    target.expected.angle,
                    target.expected.r,
                    (dist_sq as f64).sqrt()
                );
                // Blob matches an existing target - don't create a new one
                // The target will be refreshed in the normal refresh cycle
                return;
            }
        }
        drop(targets); // Release the lock before acquiring a new target

        if self.target_count() >= MAX_NUMBER_OF_TARGETS - 1 {
            log::debug!("Maximum number of targets reached, ignoring blob");
            return;
        }

        self.acquire_new_arpa_target(
            pol,
            center_pos,
            pol.time,
            TargetStatus::Acquire0,
            &Doppler::Any,
            true, // automatic: detected by guard zone/blob
        );
    }
}

impl Contour {
    fn new() -> Contour {
        Contour {
            length: 0,
            min_angle: 0,
            max_angle: 0,
            min_r: 0,
            max_r: 0,
            position: Polar::new(0, 0, 0),
            contour: Vec::new(),
        }
    }
}

impl ArpaTarget {
    pub(crate) fn new(
        position: ExtendedPosition,
        radar_pos: GeoPosition,
        uid: usize,
        spokes_per_revolution: usize,
        m_status: TargetStatus,
        have_doppler: bool,
    ) -> Self {
        // makes new target with an existing id
        Self {
            m_status,
            m_average_contour_length: 0,
            m_small_fast: false,
            m_previous_contour_length: 0,
            m_lost_count: 0,
            m_refresh_time: 0,
            m_automatic: false,
            m_radar_pos: radar_pos,
            m_course: 0.,
            m_stationary: 0,
            m_doppler_target: Doppler::Any,
            m_refreshed: RefreshState::NotFound,
            m_target_id: uid,
            m_transferred_target: false,
            m_kalman: KalmanFilter::new(spokes_per_revolution),
            contour: Contour::new(),
            m_total_pix: 0,
            m_approaching_pix: 0,
            m_receding_pix: 0,
            have_doppler,
            position,
            expected: Polar::new(0, 0, 0),
            age_rotations: 0,
            source_radar: String::new(),
            tracking_radar: String::new(),
        }
    }

    fn refresh_target_not_found(
        mut target: ArpaTarget,
        pol: Polar,
        pass: Pass,
    ) -> Result<Self, Error> {
        // target not found
        log::debug!(
            "Not found id={}, angle={}, r={}, pass={:?}, lost_count={}, status={:?}",
            target.m_target_id,
            pol.angle,
            pol.r,
            pass,
            target.m_lost_count,
            target.m_status
        );

        if target.m_small_fast && pass == Pass::Second && target.m_status == TargetStatus::Acquire2
        {
            // status 2, as it was not found,status was not increased.
            // small and fast targets MUST be found in the third sweep, and on a small distance, that is in pass 1.
            log::debug!("smallandfast set lost id={}", target.m_target_id);
            return Err(Error::Lost);
        }

        // delete low status targets immediately when not found
        if ((target.m_status == TargetStatus::Acquire1
            || target.m_status == TargetStatus::Acquire2)
            && pass == Pass::Third)
            || target.m_status == TargetStatus::Acquire0
        {
            log::debug!(
                "low status deleted id={}, angle={}, r={}, pass={:?}, lost_count={}",
                target.m_target_id,
                pol.angle,
                pol.r,
                pass,
                target.m_lost_count
            );
            return Err(Error::Lost);
        }
        if pass == Pass::Third {
            target.m_lost_count += 1;
        }

        // delete if not found too often
        if target.m_lost_count > MAX_LOST_COUNT {
            return Err(Error::Lost);
        }
        target.m_refreshed = RefreshState::NotFound;
        // Send RATTM message also for not seen messages
        /*
        if (pass == LAST_PASS && m_status > m_ri.m_target_age_to_mixer.GetValue()) {
            pol = Pos2Polar(self.position, own_pos);
            if (m_status >= m_ri.m_target_age_to_mixer.GetValue()) {
                //   f64 dist2target = pol.r / self.pixels_per_meter;
                LOG_ARPA(wxT(" pass not found as AIVDM targetid=%i"), m_target_id);
                if (m_transferred_target) {
                    //  LOG_ARPA(wxT(" passTTM targetid=%i"), m_target_id);
                    //  f64 s1 = self.position.dlat_dt;                                   // m per second
                    //  f64 s2 = self.position.dlon_dt;                                   // m  per second
                    //  m_course = rad2deg(atan2(s2, s1));
                    //  PassTTMtoOCPN(&pol, s);

                    PassAIVDMtoOCPN(&pol);
                }
                // MakeAndTransmitTargetMessage();
                // MakeAndTransmitCoT();
            }
        }
        */

        // The target wasn't found, but we do want to keep it around
        // as it may pop up on the next scan.
        target.m_transferred_target = false;
        return Ok(target);
    }

    fn refresh_target(
        mut target: ArpaTarget,
        setup: &TargetSetup,
        history: &mut HistorySpokes,
        dist: i32,
        pass: Pass,
    ) -> Result<Self, Error> {
        // refresh may be called from guard directly, better check
        let own_pos = crate::navdata::get_radar_position();
        if target.m_status == TargetStatus::Lost
            || target.m_refreshed == RefreshState::OutOfScope
            || own_pos.is_none()
        {
            return Err(Error::Lost);
        }
        if target.m_refreshed == RefreshState::Found {
            return Err(Error::AlreadyFound);
        }

        let own_pos = ExtendedPosition::new(own_pos.unwrap(), 0., 0., 0, 0., 0.);

        let mut pol = setup.pos2polar(&target.position, &own_pos);
        let alfa0 = pol.angle;
        let r0 = pol.r;
        let scan_margin = setup.scan_margin();
        let angle_time = history.spokes[setup.mod_spokes(pol.angle + scan_margin) as usize].time;
        // angle_time is the time of a spoke SCAN_MARGIN spokes forward of the target, if that spoke is refreshed we assume that the target has been refreshed

        let mut rotation_period = setup.rotation_speed_ms as u64;
        if rotation_period == 0 {
            rotation_period = 2500; // default value
        }
        if angle_time < target.m_refresh_time + rotation_period - 100 {
            // the 100 is a margin on the rotation period
            // the next image of the target is not yet there

            return Err(Error::WaitForRefresh);
        }

        // set new refresh time
        target.m_refresh_time = history.spokes[pol.angle as usize].time;
        let prev_position = target.position.clone(); // save the previous target position

        // PREDICTION CYCLE

        log::debug!(
            "Begin prediction cycle m_target_id={}, status={:?}, angle={}, r={}, contour={}, pass={:?}, lat={}, lon={}",
            target.m_target_id,
            target.m_status,
            pol.angle,
            pol.r,
            target.contour.length,
            pass,
            target.position.pos.lat,
            target.position.pos.lon
        );

        // estimated new target time
        let delta_t = if target.m_refresh_time >= prev_position.time
            && target.m_status != TargetStatus::Acquire0
        {
            (target.m_refresh_time - prev_position.time) as f64 / 1000. // in seconds
        } else {
            0.
        };

        if target.position.pos.lat > 90. || target.position.pos.lat < -90. {
            log::trace!("Target {} has unlikely latitude", target.m_target_id);
            return Err(Error::Lost);
        }

        let mut x_local = LocalPosition::new(
            GeoPosition::new(
                (target.position.pos.lat - own_pos.pos.lat) * METERS_PER_DEGREE_LATITUDE,
                (target.position.pos.lon - own_pos.pos.lon)
                    * meters_per_degree_longitude(&own_pos.pos.lat),
            ),
            target.position.dlat_dt,
            target.position.dlon_dt,
        );

        target.m_kalman.predict(&mut x_local, delta_t); // x_local is new estimated local position of the target
        // now set the polar to expected angular position from the expected local position

        pol.angle = setup.mod_spokes(
            (f64::atan2(x_local.pos.lon, x_local.pos.lat) * setup.spokes_per_revolution_f64
                / (2. * PI)) as i32,
        );
        pol.r = ((x_local.pos.lat * x_local.pos.lat + x_local.pos.lon * x_local.pos.lon).sqrt()
            * setup.pixels_per_meter) as i32;

        // zooming and target movement may  cause r to be out of bounds
        log::trace!(
            "PREDICTION m_target_id={}, pass={:?}, status={:?}, angle={}.{}, r={}.{}, contour={}, speed={}, sd_speed_kn={} doppler={:?}, lostcount={}",
            target.m_target_id,
            pass,
            target.m_status,
            alfa0,
            pol.angle,
            r0,
            pol.r,
            target.contour.length,
            target.position.speed_kn,
            target.position.sd_speed_kn,
            target.m_doppler_target,
            target.m_lost_count
        );
        if pol.r >= setup.spoke_len || pol.r <= 0 {
            // delete target if too far out
            log::trace!(
                "R out of bounds,  m_target_id={}, angle={}, r={}, contour={}, pass={:?}",
                target.m_target_id,
                pol.angle,
                pol.r,
                target.contour.length,
                pass
            );
            return Err(Error::Lost);
        }
        target.expected = pol; // save expected polar position

        // MEASUREMENT CYCLE
        // now search for the target at the expected polar position in pol
        let mut dist1 = dist;

        if pass == Pass::Third {
            // this is doubtfull $$$
            if target.m_status == TargetStatus::Acquire0
                || target.m_status == TargetStatus::Acquire1
            {
                dist1 *= 2;
            } else if target.position.speed_kn > 15. {
                dist1 *= 2;
            } /*else if (self.position.speed_kn > 30.) {
            dist1 *= 4;
            } */
        }

        let starting_position = pol;

        // here we really search for the target
        if pass == Pass::Third {
            target.m_doppler_target = Doppler::Any; // in the last pass we are not critical
        }
        let found = history.get_target(&target.m_doppler_target, pol.clone(), dist1); // main target search

        match found {
            Ok((contour, pos)) => {
                let dist_angle =
                    ((pol.angle - starting_position.angle) as f64 * pol.r as f64 / 326.) as i32;
                let dist_radial = pol.r - starting_position.r;
                let dist_total =
                    ((dist_angle * dist_angle + dist_radial * dist_radial) as f64).sqrt() as i32;

                log::debug!(
                    "id={}, Found dist_angle={}, dist_radial={}, dist_total={}, pol.angle={}, starting_position.angle={}, doppler={:?}",
                    target.m_target_id,
                    dist_angle,
                    dist_radial,
                    dist_total,
                    pol.angle,
                    starting_position.angle,
                    target.m_doppler_target
                );

                if target.m_doppler_target != Doppler::Any {
                    let backup = target.m_doppler_target;
                    target.m_doppler_target = Doppler::Any;
                    let _ = history.get_target(&target.m_doppler_target, pol.clone(), dist1); // get the contour for the target ins ANY state
                    target.pixel_counter(history);
                    target.m_doppler_target = backup;
                    let _ = history.get_target(&target.m_doppler_target, pol.clone(), dist1); // restore target in original state
                    target.state_transition(); // adapt state if required
                } else {
                    target.pixel_counter(history);
                    target.state_transition();
                }
                if target.m_average_contour_length != 0
                    && (target.contour.length < target.m_average_contour_length / 2
                        || target.contour.length > target.m_average_contour_length * 2)
                    && pass != Pass::Third
                {
                    return Err(Error::WeightedContourLengthTooHigh);
                }

                history.reset_pixels(&contour, &pos, &setup.pixels_per_meter);
                log::debug!(
                    "target Found ResetPixels m_target_id={}, angle={}, r={}, contour={}, pass={:?}, doppler={:?}",
                    target.m_target_id,
                    pol.angle,
                    pol.r,
                    target.contour.length,
                    pass,
                    target.m_doppler_target
                );
                if target.contour.length >= MAX_CONTOUR_LENGTH as i32 - 2 {
                    // don't use this blob, could be radar interference
                    // The pixels of the blob have been reset, so you won't find it again
                    log::debug!(
                        "reset found because of max contour length id={}, angle={}, r={}, contour={}, pass={:?}",
                        target.m_target_id,
                        pol.angle,
                        pol.r,
                        target.contour.length,
                        pass
                    );
                    return Err(Error::ContourLengthTooHigh);
                }

                target.m_lost_count = 0;
                let mut p_own = ExtendedPosition::empty();
                p_own.pos = history.spokes[history.mod_spokes(pol.angle) as usize].pos;
                target.age_rotations += 1;
                target.m_status = match target.m_status {
                    TargetStatus::Acquire0 => TargetStatus::Acquire1,
                    TargetStatus::Acquire1 => TargetStatus::Acquire2,
                    TargetStatus::Acquire2 => TargetStatus::Acquire3,
                    TargetStatus::Acquire3 | TargetStatus::Active => TargetStatus::Active,
                    _ => TargetStatus::Acquire0,
                };
                if target.m_status == TargetStatus::Acquire0 {
                    // as this is the first measurement, move target to measured position
                    // ExtendedPosition p_own;
                    // p_own.pos = m_ri.m_history[MOD_SPOKES(pol.angle)].pos;  // get the position at receive time
                    target.position = setup.polar2pos(&pol, &mut p_own); // using own ship location from the time of reception, only lat and lon
                    target.position.dlat_dt = 0.;
                    target.position.dlon_dt = 0.;
                    target.position.sd_speed_kn = 0.;
                    target.expected = pol;
                    log::debug!(
                        "calculated id={} pos={}",
                        target.m_target_id,
                        target.position.pos
                    );
                    target.age_rotations = 0;
                }

                // Kalman filter to  calculate the apostriori local position and speed based on found position (pol)
                if target.m_status == TargetStatus::Acquire2
                    || target.m_status == TargetStatus::Acquire3
                {
                    target.m_kalman.update_p();
                    target.m_kalman.set_measurement(
                        &mut pol,
                        &mut x_local,
                        &target.expected,
                        setup.pixels_per_meter,
                    ); // pol is measured position in polar coordinates
                }
                // x_local expected position in local coordinates

                target.position.time = pol.time; // set the target time to the newly found time, this is the time the spoke was received

                if target.m_status != TargetStatus::Acquire1 {
                    // if status == 1, then this was first measurement, keep position at measured position
                    target.position.pos.lat =
                        own_pos.pos.lat + x_local.pos.lat / METERS_PER_DEGREE_LATITUDE;
                    target.position.pos.lon = own_pos.pos.lon
                        + x_local.pos.lon / meters_per_degree_longitude(&own_pos.pos.lat);
                    target.position.dlat_dt = x_local.dlat_dt; // meters / sec
                    target.position.dlon_dt = x_local.dlon_dt; // meters /sec
                    target.position.sd_speed_kn = x_local.sd_speed_m_s * MS_TO_KN;
                }

                // Here we bypass the Kalman filter to predict the speed of the target
                // Kalman filter is too slow to adjust to the speed of (fast) new targets
                // This method however only works for targets where the accuricy of the position is high,
                // that is small targets in relation to the size of the target.

                if target.m_status == TargetStatus::Acquire2 {
                    // determine if this is a small and fast target
                    let dist_angle = pol.angle - alfa0;
                    let dist_r = pol.r - r0;
                    let size_angle = max(
                        history.mod_spokes(target.contour.max_angle - target.contour.min_angle),
                        1,
                    );
                    let size_r = max(target.contour.max_r - target.contour.min_r, 1);
                    let test = (dist_r as f64 / size_r as f64).abs()
                        + (dist_angle as f64 / size_angle as f64).abs();
                    target.m_small_fast = test > 2.;
                    log::debug!(
                        "smallandfast, id={}, test={}, dist_r={}, size_r={}, dist_angle={}, size_angle={}",
                        target.m_target_id,
                        test,
                        dist_r,
                        size_r,
                        dist_angle,
                        size_angle
                    );
                }

                const FORCED_POSITION_STATUS: u32 = 8;
                const FORCED_POSITION_AGE_FAST: u32 = 5;

                if target.m_small_fast
                    && target.age_rotations >= 2
                    && target.age_rotations < FORCED_POSITION_STATUS
                    && (target.age_rotations < FORCED_POSITION_AGE_FAST
                        || target.position.speed_kn > 10.)
                {
                    // Do a linear extrapolation of the estimated position instead of the kalman filter, as it
                    // takes too long to get up to speed for these targets.
                    let prev_pos = prev_position.pos;
                    let new_pos = setup.polar2pos(&pol, &p_own).pos;
                    let delta_lat = new_pos.lat - prev_pos.lat;
                    let delta_lon = new_pos.lon - prev_pos.lon;
                    let delta_t = pol.time - prev_position.time;
                    if delta_t > 1000 {
                        // delta_t < 1000; speed unreliable due to uncertainties in location
                        let d_lat_dt =
                            (delta_lat / (delta_t as f64)) * METERS_PER_DEGREE_LATITUDE * 1000.;
                        let d_lon_dt = (delta_lon / (delta_t as f64))
                            * meters_per_degree_longitude(&new_pos.lat)
                            * 1000.;
                        log::debug!(
                            "id={}, FORCED m_status={:?}, d_lat_dt={}, d_lon_dt={}, delta_lon_meter={}, delta_lat_meter={}, deltat={}",
                            target.m_target_id,
                            target.m_status,
                            d_lat_dt,
                            d_lon_dt,
                            delta_lon * METERS_PER_DEGREE_LATITUDE,
                            delta_lat * METERS_PER_DEGREE_LATITUDE,
                            delta_t
                        );
                        // force new position and speed, dependent of overridefactor

                        let factor: f64 = (0.8_f64).powf((target.age_rotations - 1) as f64);
                        target.position.pos.lat += factor * (new_pos.lat - target.position.pos.lat);
                        target.position.pos.lon += factor * (new_pos.lon - target.position.pos.lon);
                        target.position.dlat_dt += factor * (d_lat_dt - target.position.dlat_dt); // in meters/sec
                        target.position.dlon_dt += factor * (d_lon_dt - target.position.dlon_dt);
                        // in meters/sec
                    }
                }

                // set refresh time to the time of the spoke where the target was found
                target.m_refresh_time = target.position.time;
                if target.age_rotations >= 1 {
                    let s1 = target.position.dlat_dt; // m per second
                    let s2 = target.position.dlon_dt; // m  per second
                    target.position.speed_kn = (s1 * s1 + s2 * s2).sqrt() * MS_TO_KN; // and convert to nautical miles per hour
                    target.m_course = f64::atan2(s2, s1).to_degrees();
                    if target.m_course < 0. {
                        target.m_course += 360.;
                    }

                    log::debug!(
                        "FOUND {:?} CYCLE id={}, status={:?}, age={}, angle={}.{}, r={}.{}, contour={}, speed={}, sd_speed_kn={}, doppler={:?}",
                        pass,
                        target.m_target_id,
                        target.m_status,
                        target.age_rotations,
                        alfa0,
                        pol.angle,
                        r0,
                        pol.r,
                        target.contour.length,
                        target.position.speed_kn,
                        target.position.sd_speed_kn,
                        target.m_doppler_target
                    );

                    target.m_previous_contour_length = target.contour.length;
                    // send target data to OCPN and other radar

                    const WEIGHT_FACTOR: f64 = 0.1;

                    if target.contour.length != 0 {
                        if target.m_average_contour_length == 0 && target.contour.length != 0 {
                            target.m_average_contour_length = target.contour.length;
                        } else {
                            target.m_average_contour_length +=
                                ((target.contour.length - target.m_average_contour_length) as f64
                                    * WEIGHT_FACTOR) as i32;
                        }
                    }

                    //if (m_status >= m_ri.m_target_age_to_mixer.GetValue()) {
                    //  f64 dist2target = pol.r / self.pixels_per_meter;
                    // TODO: PassAIVDMtoOCPN(&pol);  // status s not yet used

                    // TODO: MakeAndTransmitTargetMessage();

                    // MakeAndTransmitCoT();
                    //}

                    target.m_refreshed = RefreshState::Found;
                    // A target that has been found is no longer considered a transferred target
                    target.m_transferred_target = false;
                }
            }
            Err(_e) => return Self::refresh_target_not_found(target, pol, pass),
        };
        return Ok(target);
    }

    /// Count the number of pixels in the target, and the number of approaching and receding pixels
    ///
    /// It works by moving outwards from all borders of the target until there is no target pixel
    /// at that radius. On the outside of the target this should only count 1, but on the inside
    /// it will count all pixels in the sweep until it hits the outside. The number of pixels
    /// is not fully correct: the outside pixels are counted twice.
    ///
    fn pixel_counter(&mut self, history: &HistorySpokes) {
        //  Counts total number of the various pixels in a blob
        self.m_total_pix = 0;
        self.m_approaching_pix = 0;
        self.m_receding_pix = 0;
        for i in 0..self.contour.contour.len() {
            for radius in 0..history.spokes[0].sweep.len() {
                let pixel =
                    history.spokes[history.mod_spokes(self.contour.contour[i].angle)].sweep[radius];
                let target = pixel.contains(HistoryPixel::TARGET); // above threshold bit
                if !target {
                    break;
                }
                let approaching = pixel.contains(HistoryPixel::APPROACHING); // this is Doppler approaching bit
                let receding = pixel.contains(HistoryPixel::RECEDING); // this is Doppler receding bit
                self.m_total_pix += target as u32;
                self.m_approaching_pix += approaching as u32;
                self.m_receding_pix += receding as u32;
            }
        }
    }

    /// Check doppler state of targets if Doppler is on
    fn state_transition(&mut self) {
        if !self.have_doppler || self.m_doppler_target == Doppler::AnyPlus {
            return;
        }

        let check_to_doppler = (self.m_total_pix as f64 * 0.85) as u32;
        let check_not_approaching =
            ((self.m_total_pix - self.m_approaching_pix) as f64 * 0.80) as u32;
        let check_not_receding = ((self.m_total_pix - self.m_receding_pix) as f64 * 0.80) as u32;

        let new = match self.m_doppler_target {
            Doppler::AnyDoppler | Doppler::Any => {
                // convert to APPROACHING or RECEDING
                if self.m_approaching_pix > self.m_receding_pix
                    && self.m_approaching_pix > check_to_doppler
                {
                    &Doppler::Approaching
                } else if self.m_receding_pix > self.m_approaching_pix
                    && self.m_receding_pix > check_to_doppler
                {
                    &Doppler::Receding
                } else if self.m_doppler_target == Doppler::AnyDoppler {
                    &Doppler::Any
                } else {
                    &self.m_doppler_target
                }
            }

            Doppler::Receding => {
                if self.m_receding_pix < check_not_approaching {
                    &Doppler::Any
                } else {
                    &self.m_doppler_target
                }
            }

            Doppler::Approaching => {
                if self.m_approaching_pix < check_not_receding {
                    &Doppler::Any
                } else {
                    &self.m_doppler_target
                }
            }
            _ => &self.m_doppler_target,
        };
        if *new != self.m_doppler_target {
            log::debug!(
                "Target {} Doppler state changed from {:?} to {:?}",
                self.m_target_id,
                self.m_doppler_target,
                new
            );
            self.m_doppler_target = *new;
        }
    }

    /*
        void ArpaTarget::TransferTargetToOtherRadar() {
          RadarInfo* other_radar = 0;
          LOG_ARPA(wxT("%s: TransferTargetToOtherRadar m_target_id=%i,"), m_ri.m_name, m_target_id);
          if (M_SETTINGS.radar_count != 2) {
            return;
          }
          if (!m_pi.m_radar[0] || !m_pi.m_radar[1] || !m_pi.m_radar[0].m_arpa || !m_pi.m_radar[1].m_arpa) {
            return;
          }
          if (m_pi.m_radar[0].m_state.GetValue() != RADAR_TRANSMIT || m_pi.m_radar[1].m_state.GetValue() != RADAR_TRANSMIT) {
            return;
          }
          LOG_ARPA(wxT("%s: this  radar pix/m=%f"), m_ri.m_name, self.pixels_per_meter);
          RadarInfo* long_range = m_pi.GetLongRangeRadar();
          RadarInfo* short_range = m_pi.GetShortRangeRadar();

          if (m_ri == long_range) {
            other_radar = short_range;
            int border = (int)(m_ri.m_spoke_len_max * self.pixels_per_meter / short_range.m_pixels_per_meter);
            // m_ri has largest range, other_radar smaller range. Don't transfer targets that are outside range of smaller radar
            if (m_expected.r > border) {
              // don't send small range targets to smaller radar
              return;
            }
          } else {
            other_radar = long_range;
            // this (m_ri) is the small range radar
            // we will only send larger range targets to other radar
          }
          DynamicTargetData data;
          data.target_id = m_target_id;
          data.P = m_kalman.P;
          data.position = self.position;
          LOG_ARPA(wxT("%s: lat= %f, lon= %f, m_target_id=%i,"), m_ri.m_name, self.position.pos.lat, self.position.pos.lon, m_target_id);
          data.status = m_status;
          other_radar.m_arpa.InsertOrUpdateTargetFromOtherRadar(&data, false);
        }

        void ArpaTarget::SendTargetToNearbyRadar() {
          LOG_ARPA(wxT("%s: Send target to nearby radar, m_target_id=%i,"), m_ri.m_name, m_target_id);
          RadarInfo* long_radar = m_pi.GetLongRangeRadar();
          if (m_ri != long_radar) {
            return;
          }
          DynamicTargetData data;
          data.target_id = m_target_id;
          data.P = m_kalman.P;
          data.position = self.position;
          LOG_ARPA(wxT("%s: lat= %f, lon= %f, m_target_id=%i,"), m_ri.m_name, self.position.pos.lat, self.position.pos.lon, m_target_id);
          data.status = m_status;
          if (m_pi.m_inter_radar) {
            m_pi.m_inter_radar.SendTarget(data);
            LOG_ARPA(wxT(" %s, target data send id=%i"), m_ri.m_name, m_target_id);
          }
        }

    */

    /*
    void ArpaTarget::PassAIVDMtoOCPN(Polar* pol) {
      if (!m_ri.m_AIVDMtoO.GetValue()) return;
      wxString s_TargID, s_Bear_Unit, s_Course_Unit;
      wxString s_speed, s_course, s_Dist_Unit, s_status;
      wxString s_bearing;
      wxString s_distance;
      wxString s_target_name;
      wxString nmea;

      if (m_status == LOST) return;  // AIS has no "status lost" message
      s_Bear_Unit = wxEmptyString;   // Bearing Units  R or empty
      s_Course_Unit = wxT("T");      // Course type R; Realtive T; true
      s_Dist_Unit = wxT("N");        // Speed/Distance Unit K, N, S N= NM/h = Knots

      // f64 dist = pol.r / self.pixels_per_meter / NAUTICAL_MILE.;
      f64 bearing = pol.angle * 360. / m_ri.m_spokes;
      if (bearing < 0) bearing += 360;

      int mmsi = m_target_id % 1000000;
      GeoPosition radar_pos;
      m_ri.GetRadarPosition(&radar_pos);
      f64 target_lat, target_lon;

      target_lat = self.position.pos.lat;
      target_lon = self.position.pos.lon;
      wxString result = EncodeAIVDM(mmsi, self.position.speed_kn, target_lon, target_lat, m_course);
      PushNMEABuffer(result);
      m_pi.SendToTargetMixer(result);
    }

    void ArpaTarget::PassTTMtoOCPN(Polar* pol, OCPN_target_status status) {
      // if (!m_ri.m_TTMtoO.GetValue()) return;  // also remove from conf file
      wxString s_TargID, s_Bear_Unit, s_Course_Unit;
      wxString s_speed, s_course, s_Dist_Unit, s_status;
      wxString s_bearing;
      wxString s_distance;
      wxString s_target_name;
      wxString nmea;
      char sentence[90];
      char checksum = 0;
      char* p;
      s_Bear_Unit = wxEmptyString;  // Bearing Units  R or empty
      s_Course_Unit = wxT("T");     // Course type R; Realtive T; true
      s_Dist_Unit = wxT("N");       // Speed/Distance Unit K, N, S N= NM/h = Knots
      // switch (status) {
      // case Q:
      //   s_status = wxT("Q");  // yellow
      //   break;
      // case T:
      //   s_status = wxT("T");  // green
      //   break;
      // case L:
      //   LOG_ARPA(wxT(" id=%i, status == lost"), m_target_id);
      //   s_status = wxT("L");  // ?
      //   break;
      // }

      if (m_doppler_target == ANY) {
        s_status = wxT("Q");  // yellow
      } else {
        s_status = wxT("T");
      }

      f64 dist = pol.r / self.pixels_per_meter / NAUTICAL_MILE.;
      f64 bearing = pol.angle * 360. / m_ri.m_spokes;

      if (bearing < 0) bearing += 360;
      s_TargID = wxString::Format(wxT("%2i"), m_target_id);
      s_speed = wxString::Format(wxT("%4.2f"), self.position.speed_kn);
      s_course = wxString::Format(wxT("%3.1f"), m_course);
      if (m_automatic) {
        s_target_name = wxString::Format(wxT("ARPA%2i"), m_target_id);
      } else {
        s_target_name = wxString::Format(wxT("MARPA%2i"), m_target_id);
      }
      s_distance = wxString::Format(wxT("%f"), dist);
      s_bearing = wxString::Format(wxT("%f"), bearing);

      /* Code for TTM follows. Send speed and course using TTM*/
      snprintf(sentence, sizeof(sentence), "RATTM,%2s,%s,%s,%s,%s,%s,%s, , ,%s,%s,%s, ",
               (const char*)s_TargID.mb_str(),       // 1 target id
               (const char*)s_distance.mb_str(),     // 2 Targ distance
               (const char*)s_bearing.mb_str(),      // 3 Bearing fr own ship.
               (const char*)s_Bear_Unit.mb_str(),    // 4 Brearing unit ( T = true)
               (const char*)s_speed.mb_str(),        // 5 Target speed
               (const char*)s_course.mb_str(),       // 6 Target Course.
               (const char*)s_Course_Unit.mb_str(),  // 7 Course ref T // 8 CPA Not used // 9 TCPA Not used
               (const char*)s_Dist_Unit.mb_str(),    // 10 S/D Unit N = knots/Nm
               (const char*)s_target_name.mb_str(),  // 11 Target name
               (const char*)s_status.mb_str());      // 12 Target Status L/Q/T // 13 Ref N/A

      for (p = sentence; *p; p++) {
        checksum ^= *p;
      }
      nmea.Printf(wxT("$%s*%02X\r\n"), sentence, (unsigned)checksum);
      LOG_ARPA(wxT("%s: send TTM, target=%i string=%s"), m_ri.m_name, m_target_id, nmea);
      PushNMEABuffer(nmea);
    }

    #define COPYTOMESSAGE(xxx, bitsize)                   \
      for (int i = 0; i < bitsize; i++) {                 \
        bitmessage[index - i - 1] = xxx[bitsize - i - 1]; \
      }                                                   \
      index -= bitsize;

    wxString ArpaTarget::EncodeAIVDM(int mmsi, f64 speed, f64 lon, f64 lat, f64 course) {
      // For encoding !AIVDM type 1 messages following the spec in https://gpsd.gitlab.io/gpsd/AIVDM.html
      // Sender is ecnoded as AI. There is no official identification for radar targets.

      bitset<168> bitmessage;
      int index = 168;
      bitset<6> type(1);  // 6
      COPYTOMESSAGE(type, 6);
      bitset<2> repeat(0);  // 8
      COPYTOMESSAGE(repeat, 2);
      bitset<30> mmsix(mmsi);  // 38
      COPYTOMESSAGE(mmsix, 30);
      bitset<4> navstatus(0);  // under way using engine    // 42
      COPYTOMESSAGE(navstatus, 4);
      bitset<8> rot(0);  // not turning                  // 50
      COPYTOMESSAGE(rot, 8);
      bitset<10> speedx(round(speed * 10));  // 60
      COPYTOMESSAGE(speedx, 10);
      bitset<1> accuracy(0);  // 61
      COPYTOMESSAGE(accuracy, 1);
      bitset<28> lonx(round(lon * 600000));  // 89
      COPYTOMESSAGE(lonx, 28);
      bitset<27> latx(round(lat * 600000));  // 116
      COPYTOMESSAGE(latx, 27);
      bitset<12> coursex(round(course * 10));  // COG       // 128
      COPYTOMESSAGE(coursex, 12);
      bitset<9> true_heading(511);  // 137
      COPYTOMESSAGE(true_heading, 9);
      bitset<6> timestamp(60);  // 60 means not available   // 143
      COPYTOMESSAGE(timestamp, 6);
      bitset<2> maneuvre(0);  // 145
      COPYTOMESSAGE(maneuvre, 2);
      bitset<3> spare;  // 148
      COPYTOMESSAGE(spare, 3);
      bitset<1> flags(0);  // 149
      COPYTOMESSAGE(flags, 1);
      bitset<19> rstatus(0);  // 168
      COPYTOMESSAGE(rstatus, 19);
      wxString AIVDM = "AIVDM,1,1,,A,";
      bitset<6> char_data;
      uint8_t character;
      for (int i = 168; i > 0; i -= 6) {
        for (int j = 0; j < 6; j++) {
          char_data[j] = bitmessage[i - 6 + j];
        }
        character = (uint8_t)char_data.to_ulong();
        if (character > 39) character += 8;
        character += 48;
        AIVDM += character;
      }
      AIVDM += ",0";
      // calculate checksum
      char checks = 0;
      for (size_t i = 0; i < AIVDM.length(); i++) {
        checks ^= (char)AIVDM[i];
      }
      AIVDM.Printf(wxT("!%s*%02X\r\n"), AIVDM, (unsigned)checks);
      LOG_ARPA(wxT("%s: AIS length=%i, string=%s"), m_ri.m_name, AIVDM.length(), AIVDM);
      return AIVDM;
    }

    void ArpaTarget::MakeAndTransmitCoT() {  // currently not used, CoT messages for WinTak are made bij the Targetmixer
      int mmsi = m_target_id;
      wxString mmsi_string;                                    // uid="MMSI - 001000001"
      mmsi_string.Printf(wxT(" uid=\"RADAR - %09i\""), mmsi);  // uid="MMSI - 001000002"
      wxString short_mmsi_string;
      short_mmsi_string.Printf(wxT("\"%09i\""), mmsi);
      wxDateTime dt(self.position.time);
      int year = dt.GetYear();
      int month = dt.GetMonth();
      int day = dt.GetDay();
      int hour = dt.GetHour();
      int minute = dt.GetMinute();
      int second = dt.GetSecond();
      int millisecond = dt.GetMillisecond();
      wxString speed_string;
      speed_string.Printf(wxT("\"%f\""), self.position.speed_kn);
      wxString course_string;
      course_string.Printf(wxT("\"%f\""), m_course);
    #define LIFE 4

      wxString date_time_string;        // "2022-11-12T09:29:34.784
      wxString date_time_string_stale;  // "2022-11-12T09:29:34.784
      date_time_string.Printf(wxT("\"%04i-%02i-%02iT%02i:%02i:%02i.%03i"), year, month, day, hour, minute, second, millisecond);
      date_time_string_stale.Printf(wxT("\"%04i-%02i-%02iT%02i:%02i:%02i.%03i"), year, month, day, hour, minute, second + LIFE,
                                    millisecond);
      wxString long_date_time_string;  //  time="2022-11-12T09:29:34.784000Z" start="2022-11-12T09:29:34.784000Z"
                                       //  stale="2022-11-12T09:29:34.784000Z"
      // start must be later than time, stale later than start
      long_date_time_string =
          " time=" + date_time_string + "000Z\" start=" + date_time_string + "100Z\" stale=" + date_time_string_stale + "000Z\"";
      wxString position_string;  // lat="53.461441" lon="6.178049"
      position_string.Printf(wxT(" lat=\"%f\" lon=\"%f\""), self.position.pos.lat, self.position.pos.lon);
      wxString version_string = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>";

      wxString CoTxml;
      CoTxml = version_string;
      CoTxml += "<event version=\"2.0\" type=\"a-u-S\"" + mmsi_string + " how=\"m-g\"" + long_date_time_string + ">";
      CoTxml += "<point" + position_string + " hae=\"9.0\" le=\"9.0\" ce=\"9.0\" />";
      CoTxml += "<detail" + mmsi_string + ">";
      CoTxml += "<track course=" + course_string + " speed=" + speed_string + " />";
      // CoTxml += "<contact callsign=" + short_mmsi_string + " />";
      // Remarks is not required
      // CoTxml += "<remarks>Country: Netherlands(Kingdom of the) Type : 1 MMSI : 244730001 aiscot@kees-m14.verruijt.lan</remarks>";
      CoTxml += "</detail>";
      // < _aiscot_ is not required
      // CoTxml += "<_aiscot_ cot_host_id = \"aiscot@kees-m14.verruijt.lan\" country=\"Netherlands (Kingdom of the)\" type=\"1\"
      // mmsi=\"244730002\" aton=\"False\" uscg=\"False\" crs=\"False\" />";
      CoTxml += "</event>";
      LOG_ARPA(wxT("%s: COTxml=\n%s"), m_ri.m_name, CoTxml);
      m_pi.SendToTargetMixer(CoTxml);
    }

    void ArpaTarget::MakeAndTransmitTargetMessage() {
      /* Example message
      {"target":{"uid":1,"lat":52.038339,"lon":4.111908,"sog": 2.08,"cog":39.3,
      "time":"2022-12-29T15:38:11.307000Z","stale":"2022-12-29T15:38:15.307000Z","state":5,
      "lost_count":0}}

      */

    #define LIFE_TIME_SEC 3 // life time of target after last refresh in seconds

  wxString message = wxT("{\"target\":{");

  message << wxT("\"uid\":");
  message << m_target_id;

  message << wxT(",\"source_id\":");
  message << m_pi.m_radar_id;

  message << wxT(",\"lat\":");
  message << self.position.pos.lat;
  message << wxT(",\"lon\":");
  message << self.position.pos.lon;

  wxDateTime dt(self.position.time);
  dt = dt.ToUTC();
  message << wxT(",\"time\":\"");
  message << dt.FormatISOCombined() << wxString::Format(wxT(".%03uZ\""), dt.GetMillisecond(wxDateTime::TZ::GMT0));

  dt += wxTimeSpan(0, 0, LIFE_TIME_SEC, 0);
  message << wxT(",\"stale\":\"");
  message << dt.FormatISOCombined() << wxString::Format(wxT(".%03uZ\""), dt.GetMillisecond(wxDateTime::TZ::GMT0));

  message << wxString::Format(wxT(",\"sog\":%5.2f"), self.position.speed_kn * NAUTICAL_MILE. / 3600.);
  message << wxString::Format(wxT(",\"cog\":%4.1f"), m_course);

  message << wxT(",\"state\":");
  message << m_status;
  message << wxT(",\"lost_count\":");
  message << m_lost_count;

  message << wxT("}}");

  m_pi.SendToTargetMixer(message);
}
  */

    fn set_status_lost(&mut self) {
        self.contour = Contour::new();
        self.m_previous_contour_length = 0;
        self.m_lost_count = 0;
        self.m_kalman.reset_filter();
        self.m_status = TargetStatus::Lost;
        self.m_automatic = false;
        self.m_refresh_time = 0;
        self.m_course = 0.;
        self.m_stationary = 0;
        self.position.dlat_dt = 0.;
        self.position.dlon_dt = 0.;
        self.position.speed_kn = 0.;
    }

    /// Check if this target should be broadcast to clients
    /// Manual targets are broadcast immediately from Acquire0
    /// Automatic targets wait until Acquire3 or Active to avoid noise
    pub fn should_broadcast(&self) -> bool {
        if !self.m_automatic {
            // Manual targets: broadcast all acquiring states so user sees feedback
            matches!(
                self.m_status,
                TargetStatus::Acquire0
                    | TargetStatus::Acquire1
                    | TargetStatus::Acquire2
                    | TargetStatus::Acquire3
                    | TargetStatus::Active
            )
        } else {
            // Automatic targets: only broadcast once confirmed
            matches!(
                self.m_status,
                TargetStatus::Acquire3 | TargetStatus::Active
            )
        }
    }

    /// Convert internal ArpaTarget to Signal K API format for streaming
    pub fn to_api(&self, radar_position: &GeoPosition) -> ArpaTargetApi {
        // Calculate bearing and distance from radar to target
        let dlat = self.position.pos.lat - radar_position.lat;
        let dlon = (self.position.pos.lon - radar_position.lon)
            * radar_position.lat.to_radians().cos();

        let distance = (dlat * dlat + dlon * dlon).sqrt() * METERS_PER_DEGREE_LATITUDE;
        let mut bearing = dlon.atan2(dlat); // radians
        if bearing < 0.0 {
            bearing += 2.0 * PI;
        }

        // Calculate course from velocity components (in radians)
        let mut course = self.position.dlon_dt.atan2(self.position.dlat_dt);
        if course < 0.0 {
            course += 2.0 * PI;
        }

        // Speed: convert from knots to m/s
        let speed_ms = self.position.speed_kn * 0.514444;

        ArpaTargetApi {
            id: self.m_target_id,
            status: match self.m_status {
                TargetStatus::Active => "tracking".to_string(),
                TargetStatus::Acquire3 => "acquiring".to_string(),
                TargetStatus::Lost => "lost".to_string(),
                // Acquire0, Acquire1, Acquire2 are early acquisition stages - not yet ready to show
                _ => "unknown".to_string(),
            },
            position: TargetPositionApi {
                bearing,
                distance,
                latitude: Some(self.position.pos.lat),
                longitude: Some(self.position.pos.lon),
            },
            motion: TargetMotionApi { course, speed: speed_ms },
            danger: TargetDangerApi {
                cpa: 0.0,  // TODO: implement CPA calculation
                tcpa: 0.0, // TODO: implement TCPA calculation
            },
            acquisition: if self.m_automatic {
                "auto".to_string()
            } else {
                "manual".to_string()
            },
        }
    }
}
