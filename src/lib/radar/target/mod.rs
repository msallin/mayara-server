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
pub(crate) mod spoke_coords;

pub(crate) use spoke_coords::{SpokeBearing, SpokeHeading};

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
    /// Target motion (course and speed) - omitted if both values are zero
    #[serde(skip_serializing_if = "TargetMotionApi::is_zero")]
    pub motion: TargetMotionApi,
    /// Collision danger assessment - omitted if both values are zero
    #[serde(skip_serializing_if = "TargetDangerApi::is_zero")]
    pub danger: TargetDangerApi,
    /// How target was acquired: "auto" or "manual"
    pub acquisition: String,
    /// Which guard zone acquired this target (1 or 2), or 0 for manual
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_zone: Option<u8>,
}

/// Target position in the API format
#[derive(Serialize, Clone, Debug, ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct TargetPositionApi {
    /// Bearing from radar in radians true [0, 2π)
    pub bearing: f64,
    /// Distance from radar in meters (rounded to whole meters)
    pub distance: i32,
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

impl TargetMotionApi {
    fn is_zero(&self) -> bool {
        self.course == 0.0 && self.speed == 0.0
    }
}

/// Collision danger assessment in the API format
#[derive(Serialize, Clone, Debug, ToSchema)]
pub struct TargetDangerApi {
    /// Closest Point of Approach in meters
    pub cpa: f64,
    /// Time to CPA in seconds (negative = past)
    pub tcpa: f64,
}

impl TargetDangerApi {
    fn is_zero(&self) -> bool {
        self.cpa == 0.0 && self.tcpa == 0.0
    }
}

const MIN_BLOB_PIXELS: usize = 32; // minimum number of pixels for a valid blob
const MAX_BLOB_PIXELS: usize = 10000; // maximum blob size (radar interference protection)
const MAX_LOST_COUNT: i32 = 12; // number of sweeps that target can be missed before it is set to lost
const MIN_BLOB_RANGE: i32 = 4; // ignore blobs closer than this (main bang)

// Maximum detection speed for each ARPA detect mode (in knots)
const MAX_DETECTION_SPEED_NORMAL_KN: f64 = 25.;
const MAX_DETECTION_SPEED_MEDIUM_KN: f64 = 40.;
const MAX_DETECTION_SPEED_FAST_KN: f64 = 50.;

// Legacy constants - to be removed after full migration to blob-based detection
const MIN_CONTOUR_LENGTH: usize = 6;
const MAX_CONTOUR_LENGTH: usize = 2000;

// Width of the contour line in pixels (for visibility)
const CONTOUR_WIDTH: i32 = 3;

pub const METERS_PER_DEGREE_LATITUDE: f64 = 60. * NAUTICAL_MILE_F64;
pub const KN_TO_MS: f64 = NAUTICAL_MILE_F64 / 3600.;
pub const MS_TO_KN: f64 = 3600. / NAUTICAL_MILE_F64;

const TODO_TARGET_AGE_TO_MIXER: u32 = 5;
const MAX_NUMBER_OF_TARGETS: usize = 100;

// Re-export ARPA types from the arpa module
use arpa::{ArpaDetector, BlobInProgress};

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

/// Directional offsets for contour tracing: (bearing_offset, r_offset)
const FOUR_DIRECTIONS: [(i32, i32); 4] = [
    (0, 1),  // Up (increase r)
    (1, 0),  // Right (increase bearing)
    (0, -1), // Down (decrease r)
    (-1, 0), // Left (decrease bearing)
];

/// A point used for contour tracing, with raw bearing and range values.
/// Unlike Polar, this is not normalized and allows negative values during tracing.
#[derive(Debug, Clone, Copy)]
struct ContourPoint {
    bearing: i32,
    r: i32,
}

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

    clear_contours: bool,
    auto_learn_state: i32,

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
    /// Whether to use the shared manager for target merging across radars
    /// When false, uses local storage even if shared_manager is available
    merge_targets_enabled: bool,
    /// Key identifying this radar
    radar_key: String,

    /// ARPA detector for automatic target detection via guard zones
    arpa_detector: ArpaDetector,

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
    /// Current spoke range in meters (actual data range, may differ from control range)
    spoke_range_m: u32,
    rotation_speed_ms: u32,
    stationary: bool,
    /// ARPA detect mode: 0 = Normal (25kn), 1 = Medium (40kn), 2 = Fast (50kn)
    arpa_detect_mode: i32,
}

#[derive(Debug, Clone)]
pub(crate) struct Contour {
    pub(crate) length: i32,
    /// Minimum bearing (geographic, relative to North) in raw spoke units
    min_bearing: i32,
    /// Maximum bearing (geographic, relative to North) in raw spoke units
    max_bearing: i32,
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
    pub(crate) status: TargetStatus,

    average_contour_length: i32,
    small_fast: bool,
    previous_contour_length: i32,
    lost_count: i32,
    refresh_time: u64,
    automatic: bool,
    /// Which guard zone (1 or 2) acquired this target, or 0 for manual/none
    source_zone: u8,
    radar_pos: GeoPosition,
    course: f64,
    stationary: i32,
    doppler_target: Doppler,
    pub(crate) refreshed: RefreshState,
    pub(crate) target_id: usize,
    pub(crate) transferred_target: bool,
    kalman: KalmanFilter,
    pub(crate) contour: Contour,
    total_pix: u32,
    approaching_pix: u32,
    receding_pix: u32,
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
        let start = ContourPoint {
            bearing: ang,
            r: rad,
        };

        let mut current = start; // the 4 possible translations to move from a point on the contour to the next

        let mut max_bearing = current;
        let mut min_bearing = current;
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
                    current.bearing + FOUR_DIRECTIONS[i].0,
                    current.r + FOUR_DIRECTIONS[i].1,
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

        while current.r != start.r || current.bearing != start.bearing || count == 0 {
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
                    current.bearing + FOUR_DIRECTIONS[index].0,
                    current.r + FOUR_DIRECTIONS[index].1,
                ) {
                    found = true;
                    break;
                }
                index = (index + 1) % 4;
            }
            if !found {
                return false; // no next point found (this happens when the blob consists of one single pixel)
            } // next point found
            current.bearing += FOUR_DIRECTIONS[index].0;
            current.r += FOUR_DIRECTIONS[index].1;
            if count >= length {
                return true;
            }
            count += 1;
            if current.bearing > max_bearing.bearing {
                max_bearing = current;
            }
            if current.bearing < min_bearing.bearing {
                min_bearing = current;
            }
            if current.r > max_r.r {
                max_r = current;
            }
            if current.r < min_r.r {
                min_r = current;
            }
        } // contour length is less than m_min_contour_length
        // before returning false erase this blob so we do not have to check this one again
        if min_bearing.bearing < 0 {
            min_bearing.bearing += self.spokes.len() as i32;
            max_bearing.bearing += self.spokes.len() as i32;
        }
        for a in min_bearing.bearing..=max_bearing.bearing {
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
        let mut bearing = pol.raw_bearing();
        let rad = pol.r;
        let mut limit = self.spokes.len() as i32 / 8;
        let spokes = self.spokes.len() as u32;

        if !self.pix(doppler, bearing, rad) {
            return false;
        }
        while limit >= 0 && self.pix(doppler, bearing, rad) {
            bearing -= 1;
            limit -= 1;
        }
        bearing += 1;
        pol.set_raw_bearing(bearing, spokes);

        // return true if the blob has the required min contour length
        self.multi_pix(doppler, bearing, rad)
    }

    fn pix2(&mut self, doppler: &Doppler, pol: &mut Polar, bearing: i32, r: i32) -> bool {
        let spokes = self.spokes.len() as u32;
        if self.multi_pix(doppler, bearing, r) {
            pol.set_raw_bearing(bearing, spokes);
            pol.r = r;
            return true;
        }
        return false;
    }

    /// make a search pattern along a square
    /// returns the position of the nearest blob found in pol
    /// dist is search radius (1 more or less) in radial pixels
    fn find_nearest_contour(&mut self, doppler: &Doppler, pol: &mut Polar, dist: i32) -> bool {
        let bearing = pol.raw_bearing();
        let r = pol.r;
        let distance = max(dist, 2);
        let factor: f64 = self.spokes.len() as f64 / 2.0 / PI;

        for j in 1..=distance {
            let dist_r = j;
            let dist_bearing = max((factor / r as f64 * j as f64) as i32, 1);
            // search starting from the middle
            for i in 0..=dist_bearing {
                // "upper" side
                if self.pix2(doppler, pol, bearing - i, r + dist_r) {
                    return true;
                }
                if self.pix2(doppler, pol, bearing + i, r + dist_r) {
                    return true;
                }
            }
            for i in 0..dist_r {
                // "right hand" side
                if self.pix2(doppler, pol, bearing + dist_bearing, r + i) {
                    return true;
                }
                if self.pix2(doppler, pol, bearing + dist_bearing, r - i) {
                    return true;
                }
            }
            for i in 0..=dist_bearing {
                // "lower" side
                if self.pix2(doppler, pol, bearing - i, r - dist_r) {
                    return true;
                }
                if self.pix2(doppler, pol, bearing + i, r - dist_r) {
                    return true;
                }
            }
            for i in 0..dist_r {
                // "left hand" side
                if self.pix2(doppler, pol, bearing - dist_bearing, r + i) {
                    return true;
                }
                if self.pix2(doppler, pol, bearing - dist_bearing, r - i) {
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
        let spokes = self.spokes.len() as u32;
        // Use raw bearing values for contour tracing (allows negative values)
        let mut current_bearing = current.raw_bearing();
        let mut current_r = current.r;
        let mut next_bearing: i32 = current_bearing;
        let mut next_r: i32 = current_r;

        let mut succes = false;
        let mut index = 0;

        let mut contour = Contour::new();
        contour.max_r = current_r;
        contour.max_bearing = current_bearing;
        contour.min_r = current_r;
        contour.min_bearing = current_bearing;

        // check if p inside blob
        if pol.r as usize >= self.spokes.len() {
            return Err(Error::RangeTooHigh);
        }
        if pol.r < 4 {
            return Err(Error::RangeTooLow);
        }
        if !self.pix(doppler, pol.raw_bearing(), pol.r) {
            return Err(Error::NoEchoAtStart);
        }

        // first find the orientation of border point p
        for i in 0..4 {
            index = i;
            if !self.pix(
                doppler,
                current_bearing + FOUR_DIRECTIONS[index].0,
                current_r + FOUR_DIRECTIONS[index].1,
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
                next_bearing = current_bearing + FOUR_DIRECTIONS[index].0;
                next_r = current_r + FOUR_DIRECTIONS[index].1;
                if self.pix(doppler, next_bearing, next_r) {
                    succes = true; // next point found
                    break;
                }
                index = (index + 1) % 4;
            }
            if !succes {
                return Err(Error::BrokenContour);
            }
            // next point found
            current_bearing = next_bearing;
            current_r = next_r;
            current = Polar::from_raw(current_bearing, current_r, 0, spokes);
            if count < MAX_CONTOUR_LENGTH - 1 {
                contour.contour.push(current);
            } else if count == MAX_CONTOUR_LENGTH - 1 {
                contour.contour.push(current);
                contour.contour.push(pol); // shortcut to the beginning for drawing the contour
            }
            if current_bearing > contour.max_bearing {
                contour.max_bearing = current_bearing;
            }
            if current_bearing < contour.min_bearing {
                contour.min_bearing = current_bearing;
            }
            if current_r > contour.max_r {
                contour.max_r = current_r;
            }
            if current_r < contour.min_r {
                contour.min_r = current_r;
            }
            count += 1;
        }
        contour.length = contour.contour.len() as i32;

        //  CalculateCentroid(*target);    we better use the real centroid instead of the average, TODO

        let center_bearing = self.mod_spokes((contour.max_bearing + contour.min_bearing) / 2);
        pol.set_raw_bearing(center_bearing as i32, spokes);
        contour.min_bearing = self.mod_spokes(contour.min_bearing) as i32;
        contour.max_bearing = self.mod_spokes(contour.max_bearing) as i32;
        pol.r = (contour.max_r + contour.min_r) / 2;
        pol.time = self.spokes[center_bearing].time;

        // TODO        self.radar_pos = buffer.history.spokes[pol.angle as usize].pos;

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

        let contour_found = if self.pix(doppler, pol.raw_bearing(), pol.r) {
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

        for a in contour.min_bearing - DISTANCE_BETWEEN_TARGETS
            ..=contour.max_bearing + DISTANCE_BETWEEN_TARGETS
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
            let mut max = contour.max_bearing;
            if contour.min_bearing - SHADOW_MARGIN > contour.max_bearing + SHADOW_MARGIN {
                max += self.spokes.len() as i32;
            }
            for a in contour.min_bearing - SHADOW_MARGIN..=max + SHADOW_MARGIN {
                let a = self.mod_spokes(a);
                let spoke_sweep_len = self.spokes[a].sweep.len();
                if spoke_sweep_len == 0 {
                    continue;
                }
                for r in
                    contour.max_r as usize..=min(4 * contour.max_r as usize, spoke_sweep_len - 1)
                {
                    self.spokes[a].sweep[r] =
                        self.spokes[a].sweep[r].intersection(HistoryPixel::BACKUP);
                    // also clear both Doppler bits
                }
            }
        }

        // Draw the contour in the history. This is copied to the output data
        // on the next sweep. Widen the contour for better visibility.
        let half_width = CONTOUR_WIDTH / 2;
        for p in &contour.contour {
            // Set CONTOUR bits for pixels within CONTOUR_WIDTH in both bearing and range
            for db in -half_width..=half_width {
                for dr in -half_width..=half_width {
                    let bearing_idx = self.mod_spokes(p.raw_bearing() + db);
                    let r = p.r + dr;
                    let spoke_sweep_len = self.spokes[bearing_idx].sweep.len();
                    if r >= 0 && (r as usize) < spoke_sweep_len {
                        self.spokes[bearing_idx].sweep[r as usize].insert(HistoryPixel::CONTOUR);
                    }
                }
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
                                       * pol.bearing_in_rad(self.spokes_per_revolution_f64).cos()
            / METERS_PER_DEGREE_LATITUDE;
        pos.pos.lon += (pol.r as f64 / self.pixels_per_meter)  // Scale to fraction of distance to radar
                                       * pol.bearing_in_rad(self.spokes_per_revolution_f64).sin()
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
        return Polar::from_raw(angle as i32, r, p.time, self.spokes_per_revolution as u32);
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
                spoke_range_m: 0,

                rotation_speed_ms: 0,
                stationary,
                arpa_detect_mode: 0, // Default to Normal mode (25kn)
            },
            next_target_id: 0,
            arpa_via_doppler: false,

            history: HistorySpokes::new(stationary, spokes_per_revolution, spoke_len),
            targets: Arc::new(RwLock::new(HashMap::new())),
            clear_contours: false,
            auto_learn_state: 0,

            course: 0.,
            course_weight: 0,
            course_samples: 0,

            scanned_angle: -1,
            refreshed_angle: -1,

            shared_manager,
            merge_targets_enabled: false, // Disabled by default, enabled via MergeTargets control
            radar_key,

            arpa_detector: ArpaDetector::new(spokes_per_revolution),

            sk_client_tx,
        }
    }

    /// Enable or disable target merging across multiple radars
    /// When enabled and a shared_manager is available, targets are stored in
    /// the shared manager and can be tracked by any radar with coverage.
    /// When disabled, targets are stored locally per-radar.
    pub fn set_merge_targets(&mut self, enabled: bool) {
        self.merge_targets_enabled = enabled;
        log::info!(
            "Target merging {}: shared_manager={}",
            if enabled { "enabled" } else { "disabled" },
            self.shared_manager.is_some()
        );
    }

    /// Check if we should use the shared target manager
    /// Returns true only if both merging is enabled AND a shared manager exists
    fn use_shared_manager(&self) -> bool {
        self.merge_targets_enabled && self.shared_manager.is_some()
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
            delta.add_target_update(&self.radar_key, target.target_id, Some(target_api));
            // Ignore send errors - no receivers is normal when no clients connected
            let _ = tx.send(delta);
        }
    }

    /// Broadcast that a target was deleted (sends null to remove from client)
    fn broadcast_target_deleted(&self, target_id: usize) {
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
            self.arpa_detector.guard_zones[zone_index].update_from_config(
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

    /// Set ARPA detect mode: 0 = Normal (25kn), 1 = Medium (40kn), 2 = Fast (50kn)
    pub fn set_arpa_detect_mode(&mut self, mode: i32) {
        self.setup.arpa_detect_mode = mode;
        log::info!(
            "ARPA detect mode set to {}",
            match mode {
                0 => "Normal (25kn)",
                1 => "Medium (40kn)",
                2 => "Fast (50kn)",
                _ => "Unknown",
            }
        );
    }

    fn reset_history(&mut self) {
        self.history = HistorySpokes::new(
            self.setup.stationary,
            self.setup.spokes_per_revolution,
            self.setup.spoke_len,
        );
        // Clear any blobs in progress since history was reset
        self.arpa_detector.complete_all_blobs();
    }

    fn clear_contours(&mut self) {
        for (_, t) in self.targets.write().unwrap().iter_mut() {
            t.contour.length = 0;
            t.average_contour_length = 0;
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
            if target.status != TargetStatus::Lost {
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
        // Use shared manager only if merging is enabled
        if self.use_shared_manager() {
            if let Some(ref manager) = self.shared_manager {
                if manager.delete_target_near(pos).is_none() {
                    log::debug!(
                        "Could not find (M)ARPA target to delete within 1000 meters from {}",
                        pos
                    );
                }
            }
        } else {
            // Local storage mode
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

        // Use shared manager only if merging is enabled
        if self.use_shared_manager() {
            if let Some(ref manager) = self.shared_manager {
                manager.acquire_target(
                    &self.radar_key,
                    target_pos,
                    Doppler::Any,
                    self.setup.spokes_per_revolution as usize,
                    self.setup.have_doppler,
                );
            }
        } else {
            // Local storage mode
            let id = self.get_next_target_id();

            // Get current radar position for calculating polar coordinates
            let radar_pos =
                crate::navdata::get_radar_position().unwrap_or(GeoPosition::new(0., 0.));

            let mut target = ArpaTarget::new(
                target_pos.clone(),
                radar_pos.clone(),
                id,
                self.setup.spokes_per_revolution as usize,
                status,
                self.setup.have_doppler,
            );

            // Calculate polar coordinates so refresh can find the target
            // This is critical - without proper polar coords, the target won't be found
            if self.setup.pixels_per_meter > 0.0 {
                let own_pos = ExtendedPosition::new(radar_pos, 0., 0., target_pos.time, 0., 0.);
                let polar = self.setup.pos2polar(&target_pos, &own_pos);
                target.contour.position = polar.clone();
                target.expected = polar.clone();

                log::info!(
                    "MARPA target {} acquired: bearing={}, r={}, lat={:.6}, lon={:.6}",
                    id,
                    polar.bearing,
                    polar.r,
                    target_pos.pos.lat,
                    target_pos.pos.lon
                );
            } else {
                log::warn!(
                    "MARPA target {} acquired without valid pixels_per_meter ({}), tracking may fail",
                    id,
                    self.setup.pixels_per_meter
                );
            }

            target.source_radar = self.radar_key.clone();
            target.tracking_radar = self.radar_key.clone();
            self.targets.write().unwrap().insert(id, target);
        }
    }

    fn cleanup_lost_targets(&mut self) {
        // Collect IDs of lost targets to broadcast deletion
        let lost_target_ids: Vec<usize> = if self.use_shared_manager() {
            self.shared_manager
                .as_ref()
                .map(|m| m.get_lost_target_ids())
                .unwrap_or_default()
        } else {
            self.targets
                .read()
                .unwrap()
                .iter()
                .filter(|(_, t)| t.status == TargetStatus::Lost)
                .map(|(id, _)| *id)
                .collect()
        };

        // Broadcast deletion for each lost target
        for target_id in lost_target_ids {
            self.broadcast_target_deleted(target_id);
        }

        // Cleanup via shared manager if merging is enabled
        if self.use_shared_manager() {
            if let Some(ref manager) = self.shared_manager {
                manager.cleanup_lost_targets();
            }
        }

        // Always cleanup local targets too
        self.targets
            .write()
            .unwrap()
            .retain(|_, t| t.status != TargetStatus::Lost);
        for (_, v) in self.targets.write().unwrap().iter_mut() {
            v.refreshed = RefreshState::NotFound;
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
            // Log target bearings for each segment refresh
            let target_bearings: Vec<SpokeBearing> = self
                .targets
                .read()
                .unwrap()
                .values()
                .map(|t| t.contour.position.bearing)
                .collect();
            log::debug!(
                "refresh_all_arpa_targets: {} targets at bearings {:?}, scanning range {}..{}",
                target_count,
                target_bearings,
                start_angle,
                end_angle
            );
        }

        self.cleanup_lost_targets();

        // Main target refresh loop
        //
        // Calculate the maximum search radius based on the ARPA detect mode.
        // This is the distance (in pixels) a target could travel during one radar rotation.
        // Normal: 25kn max, Medium: 40kn max, Fast: 50kn max
        let max_speed_kn = match self.setup.arpa_detect_mode {
            2 => MAX_DETECTION_SPEED_FAST_KN,
            1 => MAX_DETECTION_SPEED_MEDIUM_KN,
            _ => MAX_DETECTION_SPEED_NORMAL_KN,
        };
        let speed = max_speed_kn * KN_TO_MS; // m/sec
        let rotation_ms = if self.setup.rotation_speed_ms > 0 {
            self.setup.rotation_speed_ms
        } else {
            2500 // fallback if not yet computed
        };
        let search_radius =
            (speed * rotation_ms as f64 * self.setup.pixels_per_meter / 1000.) as i32;

        // Use shared manager for targets if merging is enabled
        if self.use_shared_manager() {
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
                return;
            }
        }

        // Local storage mode
        let target_ids: Vec<usize> = self.targets.read().unwrap().keys().cloned().collect();
        log::debug!(
            "refresh_all_arpa_targets: local mode, {} target_ids: {:?}",
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
                "Processing local target {} at bearing {}",
                target_id,
                target.contour.position.bearing
            );
            self.refresh_single_target(target_id, target, start_angle, end_angle, search_radius);
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
        let spokes = self.setup.spokes_per_revolution as u32;
        if !target.contour.position.bearing_is_between(
            SpokeBearing::new(start_angle, spokes),
            SpokeBearing::new(end_angle, spokes),
            spokes,
        ) {
            return;
        }

        log::info!(
            "Refreshing target {}: status={:?}, bearing={}, range={}..{}",
            target_id,
            target.status,
            target.contour.position.bearing,
            start_angle,
            end_angle
        );

        // Three-pass search strategy with expanding search radius:
        // - Pass::First:  search_radius/4 - tight search, only for confirmed fast targets
        // - Pass::Second: search_radius/3 - medium search for all targets
        // - Pass::Third:  full search_radius - wide search to catch fast-moving targets
        //
        // The Kalman filter predicts where the target should be, then we search outward
        // from that predicted position. Starting with a small radius reduces false matches
        // on nearby blobs. If not found, we expand the search in subsequent passes.
        for pass in Pass::iter() {
            let radius = match pass {
                Pass::First => search_radius / 4,
                Pass::Second => search_radius / 3,
                Pass::Third => search_radius,
            };

            // Pass::First only runs for targets that are already confirmed as moving fast,
            // or when autolearn is ready. This avoids wasting time on slow/stationary targets.
            if pass == Pass::First
                && !((target.position.speed_kn >= 2.5
                    && target.age_rotations >= TODO_TARGET_AGE_TO_MIXER)
                    || self.auto_learn_state >= 1)
            {
                continue;
            }

            let prev_status = target.status.clone();
            let clone = target.clone();
            match ArpaTarget::refresh_target(clone, &self.setup, &mut self.history, radius, pass) {
                Ok(t) => {
                    // Log status changes
                    if t.status != prev_status {
                        log::info!(
                            "Target {} status: {:?} -> {:?}, speed={:.1}kn, lost_count={}",
                            t.target_id,
                            prev_status,
                            t.status,
                            t.position.speed_kn,
                            t.lost_count
                        );
                    }
                    target = t;
                }
                Err(e) => match e {
                    Error::Lost => {
                        log::info!("Target {} lost", target.target_id);
                        target.status = TargetStatus::Lost;
                    }
                    _ => {
                        log::debug!("Target {} refresh error {:?}", target.target_id, e);
                    }
                },
            }
        }

        // Broadcast target state to SignalK clients
        // When lost, send update with status="lost" - deletion (null) is sent in cleanup_lost_targets
        self.broadcast_target_update(&target, &target.radar_pos);

        // Update the target back to storage
        if self.use_shared_manager() {
            if let Some(ref manager) = self.shared_manager {
                manager.update_target(target_id, target, self.history.spokes[0].time);
            }
        } else {
            self.targets.write().unwrap().insert(target_id, target);
        }
    }

    pub fn delete_all_targets(&mut self) {
        // Broadcast lost for all targets before deleting
        let target_ids: Vec<usize> = if self.use_shared_manager() {
            self.shared_manager
                .as_ref()
                .map(|m| m.get_all_target_ids())
                .unwrap_or_default()
        } else {
            self.targets.read().unwrap().keys().copied().collect()
        };
        for target_id in target_ids {
            self.broadcast_target_deleted(target_id);
        }

        // Use shared manager if merging is enabled
        if self.use_shared_manager() {
            if let Some(ref manager) = self.shared_manager {
                manager.delete_all_targets();
            }
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
        //LOG_ARPA(wxT("%s: InsertOrUpdateTarget id=%i, found=%i"), m_ri.m_name, uid, (*target).target_id);
        if ((*target).target_id == uid) {  // target found!
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
          updated_target.refresh_time = m_ri.m_history[MOD_SPOKES(pol.angle)].time;
        }
      }
      //LOG_ARPA(wxT("%s: InsertOrUpdateTarget processing id=%i"), m_ri.m_name, uid);
      updated_target.kalman.P = data->P;
      updated_target.m_position = data->position;
      updated_target.status = data->status;
      LOG_ARPA(wxT("%s: transferred id=%i, lat= %f, lon= %f, status=%i,"), m_ri.m_name, updated_target.target_id,
               updated_target.m_position.pos.lat, updated_target.m_position.pos.lon, updated_target.status);
      updated_target.doppler_target = ANY;
      updated_target.lost_count = 0;
      updated_target.automatic = true;
      double s1 = updated_target.m_position.dlat_dt;  // m per second
      double s2 = updated_target.m_position.dlon_dt;                                   // m  per second
      updated_target.course = rad2deg(atan2(s2, s1));
      if (remote) {   // inserted or updated target originated from another radar
        updated_target.transferred_target = true;
        //LOG_ARPA(wxT(" transferred_target = true targetid=%i"), updated_target.target_id);
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
      target.doppler_target = doppl;
      target.m_position = target_pos;  // Expected position
      target.m_position.time = wxGetUTCTimeMillis();
      target.m_position.dlat_dt = 0.;
      target.m_position.dlon_dt = 0.;
      target.m_position.speed_kn = 0.;
      target.m_position.sd_speed_kn = 0.;
      target.status = status;
      target.m_max_angle.angle = 0;
      target.m_min_angle.angle = 0;
      target.m_max_r.r = 0;
      target.m_min_r.r = 0;
      target.doppler_target = doppl;
      target.refreshed = NOT_FOUND;
      target.automatic = true;
      target->RefreshTarget(TARGET_SEARCH_RADIUS1, 1);

      m_targets.push_back(std::move(target));
      return true;
    }

    void RadarArpa::ClearContours() { clear_contours = true; }
    */

    /*
    void RadarArpa::ProcessIncomingMessages() {
      DynamicTargetData* target;
      if (clear_contours) {
        clear_contours = false;
        for (auto target = m_targets.begin(); target != m_targets.end(); target++) {
          (*target).m_contour_length = 0;
          (*target).previous_contour_length = 0;
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
        source_zone: u8,
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
        target.automatic = automatic;
        target.source_zone = source_zone;

        log::info!(
            "Target {} acquired: status={:?}, bearing={}, range={}, lat={:.6}, lon={:.6}",
            uid,
            target.status,
            pol.bearing,
            pol.r,
            target_pos.pos.lat,
            target_pos.pos.lon
        );

        // Broadcast new target to SignalK clients
        self.broadcast_target_update(&target, &target.radar_pos);

        // Store to shared manager if merging is enabled, otherwise local storage
        if self.use_shared_manager() {
            if let Some(ref manager) = self.shared_manager {
                manager.add_target(uid, target, &self.radar_key);
            }
        } else {
            self.targets.write().unwrap().insert(uid, target);
        }
    }

    /// Work on the targets when spoke `angle` has just been processed.
    /// We look for targets a while back, so one quarter rotation ago.
    /// Get the number of targets currently being tracked
    fn target_count(&self) -> usize {
        if self.use_shared_manager() {
            self.shared_manager
                .as_ref()
                .map(|m| m.target_count())
                .unwrap_or(0)
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
            self.setup.spoke_range_m = spoke.range;
            self.reset_history();
            self.clear_contours();

            // Recalculate guard zones when pixels_per_meter changes
            self.arpa_detector
                .recalculate_zones(self.setup.spokes_per_revolution, pixels_per_meter);
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

        // Store heading for use by ARPA blob processing
        let spokes_u32 = spokes as u32;
        self.arpa_detector.current_heading = SpokeHeading::new(heading, spokes_u32);

        // Build up static background when in stationary mode
        let background_on = self.setup.stationary;

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
        let bearing = SpokeBearing::new(angle as i32, spokes_u32);
        let heading_typed = SpokeHeading::new(heading, spokes_u32);
        self.process_blob_pixels(bearing, heading_typed, &strong_pixels, time, pos.clone());

        self.refresh_targets(angle);
    }

    /// Overlay blob edge markers from history onto the spoke data
    /// In stationary mode, also displays the static background in light grey
    fn overlay_blob_edges(&self, spoke: &mut Spoke, angle: usize, legend: &Legend) {
        if angle >= self.history.spokes.len() {
            return;
        }

        // In stationary mode, show stationary layer in light grey
        if self.setup.stationary {
            if let Some(layer) = &self.history.stationary_layer {
                // Light grey color for static background (using a low value that won't be interpreted as a target)
                let static_bg_color = legend.static_background.unwrap_or(32);
                for radius in 0..spoke.data.len().min(layer.ncols()) {
                    // Show pixels that have been seen at least a few times
                    if layer[[angle, radius]] >= 3 {
                        // Only show static background if current pixel is not already a return
                        if spoke.data[radius] < legend.weakest_return.unwrap_or(128) {
                            spoke.data[radius] = static_bg_color;
                        }
                    }
                }
            }
        }

        let sweep = &self.history.spokes[angle].sweep;
        let blob_edge_color = legend.target_border;

        for (radius, pixel) in sweep.iter().enumerate() {
            if pixel.contains(HistoryPixel::CONTOUR) && radius < spoke.data.len() {
                spoke.data[radius] = blob_edge_color;
            }
        }
    }

    /// Process strong pixels from a spoke and update blobs in progress.
    /// Delegates to ArpaDetector for automatic target detection within guard zones.
    fn process_blob_pixels(
        &mut self,
        bearing: SpokeBearing,
        heading: SpokeHeading,
        strong_pixels: &[i32],
        time: u64,
        pos: GeoPosition,
    ) {
        let spokes = self.setup.spokes_per_revolution as u32;

        // Delegate blob detection to ArpaDetector
        let completed_blobs = self.arpa_detector.process_blob_pixels(
            bearing,
            heading,
            strong_pixels,
            time,
            pos,
            spokes,
        );

        // Process each completed blob for target acquisition
        for blob in completed_blobs {
            self.process_completed_blob(blob);
        }
    }

    /// Process a completed blob - check validity and pass to target acquisition
    fn process_completed_blob(&mut self, blob: BlobInProgress) {
        let spokes = self.setup.spokes_per_revolution as u32;

        if !blob.is_valid() {
            log::trace!(
                "Blob rejected: pixels={}, range={}..{}, bearings={}..{}",
                blob.pixel_count,
                blob.min_r,
                blob.max_r,
                blob.min_bearing,
                blob.max_bearing
            );
            return;
        }

        let (center_bearing_raw, center_r) = blob.center(spokes);
        let center_bearing = SpokeBearing::new(center_bearing_raw, spokes);

        // Use the heading from when the blob was first detected, not the current heading
        // This ensures consistent guard zone validation since the blob's pixels were
        // verified against guard zones using this heading when they were collected
        let heading = blob.start_heading;

        // Determine which guard zone (if any) contains this blob
        let source_zone =
            self.arpa_detector
                .get_containing_zone(center_bearing, center_r, heading, spokes);

        // Verify blob is within a guard zone - this should always pass since pixels
        // were filtered by guard zone during collection, but log details if it fails
        if source_zone == 0 {
            let current_heading = self.arpa_detector.current_heading;
            log::warn!(
                "Blob outside guard zones! center=({}, {}), start_heading={}, current_heading={}, zone0={:?}, zone1={:?}",
                center_bearing,
                center_r,
                heading,
                current_heading,
                (
                    self.arpa_detector.guard_zones[0].start_angle,
                    self.arpa_detector.guard_zones[0].end_angle,
                    self.arpa_detector.guard_zones[0].inner_range,
                    self.arpa_detector.guard_zones[0].outer_range
                ),
                (
                    self.arpa_detector.guard_zones[1].start_angle,
                    self.arpa_detector.guard_zones[1].end_angle,
                    self.arpa_detector.guard_zones[1].inner_range,
                    self.arpa_detector.guard_zones[1].outer_range
                )
            );
            return;
        }

        // Get the ship position at the center bearing from history
        // This is more accurate than blob.start_pos which is from when the blob started
        let center_bearing_idx = (center_bearing_raw as usize) % self.history.spokes.len();
        let center_pos = self.history.spokes[center_bearing_idx].pos.clone();
        let center_time = self.history.spokes[center_bearing_idx].time;

        // Comprehensive logging for range debugging
        let gz0 = &self.arpa_detector.guard_zones[0];
        let gz1 = &self.arpa_detector.guard_zones[1];
        log::info!(
            "Blob detected: spoke_range={}m, spoke_len={}, ppm={:.4}, gz0_config={}..{}m, gz0_px={}..{}, gz1_config={}..{}m, gz1_px={}..{}, blob_center_r={}, blob_r={}..{}, zone={}",
            self.setup.spoke_range_m,
            self.setup.spoke_len,
            self.setup.pixels_per_meter,
            gz0.config_inner_range_m as i32,
            gz0.config_outer_range_m as i32,
            gz0.inner_range,
            gz0.outer_range,
            gz1.config_inner_range_m as i32,
            gz1.config_outer_range_m as i32,
            gz1.inner_range,
            gz1.outer_range,
            center_r,
            blob.min_r,
            blob.max_r,
            source_zone
        );

        // Create a Polar position for the blob center
        let pol = Polar::new(center_bearing, center_r, center_time);

        // Trace the real contour and set CONTOUR bits for visualization
        let dist = center_r / 2;
        if let Ok((contour, _)) = self.history.get_target(&Doppler::Any, pol.clone(), dist) {
            self.history
                .reset_pixels(&contour, &pol, &self.setup.pixels_per_meter);
        }

        // Pass to target acquisition with the ship position at the center angle
        self.acquire_or_match_blob(pol, center_pos, source_zone);
    }

    /// Try to match blob to existing target, or acquire as new target
    fn acquire_or_match_blob(&mut self, pol: Polar, center_pos: GeoPosition, source_zone: u8) {
        // Search radius for matching blobs to targets (in pixels)
        // This should be larger than typical target movement between sweeps
        const MATCH_RADIUS: i32 = 20;

        // Check if blob matches any existing target's expected position
        let targets = self.targets.read().unwrap();
        let spokes = self.setup.spokes_per_revolution;

        for (target_id, target) in targets.iter() {
            if target.status == TargetStatus::Lost {
                continue;
            }

            // Calculate distance from blob center to target's expected position
            let bearing_diff = (pol.raw_bearing() - target.expected.raw_bearing()).abs();
            let bearing_diff = bearing_diff.min(spokes - bearing_diff); // Handle wrap-around
            let range_diff = (pol.r - target.expected.r).abs();

            // Check if within search radius (use squared distance for efficiency)
            let dist_sq = bearing_diff * bearing_diff + range_diff * range_diff;
            if dist_sq <= MATCH_RADIUS * MATCH_RADIUS {
                log::debug!(
                    "Blob matches existing target {}: blob=({},{}), expected=({},{}), dist={}",
                    target_id,
                    pol.bearing,
                    pol.r,
                    target.expected.bearing,
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
            true,        // automatic: detected by guard zone/blob
            source_zone, // which guard zone detected this target
        );
    }
}

impl Contour {
    fn new() -> Contour {
        Contour {
            length: 0,
            min_bearing: 0,
            max_bearing: 0,
            min_r: 0,
            max_r: 0,
            position: Polar::zero(),
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
        status: TargetStatus,
        have_doppler: bool,
    ) -> Self {
        // makes new target with an existing id
        Self {
            status,
            average_contour_length: 0,
            small_fast: false,
            previous_contour_length: 0,
            lost_count: 0,
            refresh_time: 0,
            automatic: false,
            source_zone: 0,
            radar_pos: radar_pos,
            course: 0.,
            stationary: 0,
            doppler_target: Doppler::Any,
            refreshed: RefreshState::NotFound,
            target_id: uid,
            transferred_target: false,
            kalman: KalmanFilter::new(spokes_per_revolution),
            contour: Contour::new(),
            total_pix: 0,
            approaching_pix: 0,
            receding_pix: 0,
            have_doppler,
            position,
            expected: Polar::zero(),
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
            "Not found id={}, bearing={}, r={}, pass={:?}, lost_count={}, status={:?}",
            target.target_id,
            pol.bearing,
            pol.r,
            pass,
            target.lost_count,
            target.status
        );

        if target.small_fast && pass == Pass::Second && target.status == TargetStatus::Acquire2 {
            // status 2, as it was not found,status was not increased.
            // small and fast targets MUST be found in the third sweep, and on a small distance, that is in pass 1.
            log::debug!("smallandfast set lost id={}", target.target_id);
            return Err(Error::Lost);
        }

        // delete low status targets immediately when not found
        if ((target.status == TargetStatus::Acquire1 || target.status == TargetStatus::Acquire2)
            && pass == Pass::Third)
            || target.status == TargetStatus::Acquire0
        {
            log::debug!(
                "low status deleted id={}, bearing={}, r={}, pass={:?}, lost_count={}",
                target.target_id,
                pol.bearing,
                pol.r,
                pass,
                target.lost_count
            );
            return Err(Error::Lost);
        }
        if pass == Pass::Third {
            target.lost_count += 1;
        }

        // delete if not found too often
        if target.lost_count > MAX_LOST_COUNT {
            return Err(Error::Lost);
        }
        target.refreshed = RefreshState::NotFound;
        // Send RATTM message also for not seen messages
        /*
        if (pass == LAST_PASS && status > m_ri.m_target_age_to_mixer.GetValue()) {
            pol = Pos2Polar(self.position, own_pos);
            if (status >= m_ri.m_target_age_to_mixer.GetValue()) {
                //   f64 dist2target = pol.r / self.pixels_per_meter;
                LOG_ARPA(wxT(" pass not found as AIVDM targetid=%i"), target_id);
                if (transferred_target) {
                    //  LOG_ARPA(wxT(" passTTM targetid=%i"), target_id);
                    //  f64 s1 = self.position.dlat_dt;                                   // m per second
                    //  f64 s2 = self.position.dlon_dt;                                   // m  per second
                    //  course = rad2deg(atan2(s2, s1));
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
        target.transferred_target = false;
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
        if target.status == TargetStatus::Lost
            || target.refreshed == RefreshState::OutOfScope
            || own_pos.is_none()
        {
            return Err(Error::Lost);
        }
        if target.refreshed == RefreshState::Found {
            return Err(Error::AlreadyFound);
        }

        let own_pos = ExtendedPosition::new(own_pos.unwrap(), 0., 0., 0, 0., 0.);

        let mut pol = setup.pos2polar(&target.position, &own_pos);
        let bearing0 = pol.raw_bearing();
        let r0 = pol.r;
        let scan_margin = setup.scan_margin();
        let bearing_time =
            history.spokes[setup.mod_spokes(pol.raw_bearing() + scan_margin) as usize].time;
        // bearing_time is the time of a spoke SCAN_MARGIN spokes forward of the target, if that spoke is refreshed we assume that the target has been refreshed

        let mut rotation_period = setup.rotation_speed_ms as u64;
        if rotation_period == 0 {
            rotation_period = 2500; // default value
        }
        if bearing_time < target.refresh_time + rotation_period - 100 {
            // the 100 is a margin on the rotation period
            // the next image of the target is not yet there

            return Err(Error::WaitForRefresh);
        }

        // set new refresh time
        target.refresh_time = history.spokes[pol.raw_bearing() as usize].time;
        let prev_position = target.position.clone(); // save the previous target position

        // PREDICTION CYCLE

        log::debug!(
            "Begin prediction cycle target_id={}, status={:?}, bearing={}, r={}, contour={}, pass={:?}, lat={}, lon={}",
            target.target_id,
            target.status,
            pol.bearing,
            pol.r,
            target.contour.length,
            pass,
            target.position.pos.lat,
            target.position.pos.lon
        );

        // estimated new target time
        let delta_t = if target.refresh_time >= prev_position.time
            && target.status != TargetStatus::Acquire0
        {
            (target.refresh_time - prev_position.time) as f64 / 1000. // in seconds
        } else {
            0.
        };

        if target.position.pos.lat > 90. || target.position.pos.lat < -90. {
            log::trace!("Target {} has unlikely latitude", target.target_id);
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

        // Set Kalman filter noise based on ARPA detect mode:
        // Normal (0): 0.015 - straighter paths, max 25kn
        // Medium (1): 0.1 - moderate maneuvering, max 40kn
        // Fast (2): 1.0 - aggressive maneuvering, max 50kn
        let noise = match setup.arpa_detect_mode {
            2 => KalmanFilter::noise_fast(),
            1 => KalmanFilter::noise_medium(),
            _ => KalmanFilter::noise_normal(),
        };
        target.kalman.set_noise(noise);

        target.kalman.predict(&mut x_local, delta_t); // x_local is new estimated local position of the target
        // now set the polar to expected angular position from the expected local position

        let new_bearing = setup.mod_spokes(
            (f64::atan2(x_local.pos.lon, x_local.pos.lat) * setup.spokes_per_revolution_f64
                / (2. * PI)) as i32,
        ) as i32;
        pol.set_raw_bearing(new_bearing, setup.spokes_per_revolution as u32);
        pol.r = ((x_local.pos.lat * x_local.pos.lat + x_local.pos.lon * x_local.pos.lon).sqrt()
            * setup.pixels_per_meter) as i32;

        // zooming and target movement may  cause r to be out of bounds
        log::trace!(
            "PREDICTION target_id={}, pass={:?}, status={:?}, bearing={}.{}, r={}.{}, contour={}, speed={}, sd_speed_kn={} doppler={:?}, lostcount={}",
            target.target_id,
            pass,
            target.status,
            bearing0,
            pol.bearing,
            r0,
            pol.r,
            target.contour.length,
            target.position.speed_kn,
            target.position.sd_speed_kn,
            target.doppler_target,
            target.lost_count
        );
        if pol.r >= setup.spoke_len || pol.r <= 0 {
            // delete target if too far out
            log::trace!(
                "R out of bounds,  target_id={}, bearing={}, r={}, contour={}, pass={:?}",
                target.target_id,
                pol.bearing,
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
            if target.status == TargetStatus::Acquire0 || target.status == TargetStatus::Acquire1 {
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
            target.doppler_target = Doppler::Any; // in the last pass we are not critical
        }
        let found = history.get_target(&target.doppler_target, pol.clone(), dist1); // main target search

        match found {
            Ok((contour, pos)) => {
                let dist_bearing = ((pol.raw_bearing() - starting_position.raw_bearing()) as f64
                    * pol.r as f64
                    / 326.) as i32;
                let dist_radial = pol.r - starting_position.r;
                let dist_total = ((dist_bearing * dist_bearing + dist_radial * dist_radial) as f64)
                    .sqrt() as i32;

                log::debug!(
                    "id={}, Found dist_bearing={}, dist_radial={}, dist_total={}, pol.bearing={}, starting_position.bearing={}, doppler={:?}",
                    target.target_id,
                    dist_bearing,
                    dist_radial,
                    dist_total,
                    pol.bearing,
                    starting_position.bearing,
                    target.doppler_target
                );

                if target.doppler_target != Doppler::Any {
                    let backup = target.doppler_target;
                    target.doppler_target = Doppler::Any;
                    let _ = history.get_target(&target.doppler_target, pol.clone(), dist1); // get the contour for the target ins ANY state
                    target.pixel_counter(history);
                    target.doppler_target = backup;
                    let _ = history.get_target(&target.doppler_target, pol.clone(), dist1); // restore target in original state
                    target.state_transition(); // adapt state if required
                } else {
                    target.pixel_counter(history);
                    target.state_transition();
                }
                if target.average_contour_length != 0
                    && (target.contour.length < target.average_contour_length / 2
                        || target.contour.length > target.average_contour_length * 2)
                    && pass != Pass::Third
                {
                    return Err(Error::WeightedContourLengthTooHigh);
                }

                history.reset_pixels(&contour, &pos, &setup.pixels_per_meter);
                log::debug!(
                    "target Found ResetPixels target_id={}, bearing={}, r={}, contour={}, pass={:?}, doppler={:?}",
                    target.target_id,
                    pol.bearing,
                    pol.r,
                    target.contour.length,
                    pass,
                    target.doppler_target
                );
                if target.contour.length >= MAX_CONTOUR_LENGTH as i32 - 2 {
                    // don't use this blob, could be radar interference
                    // The pixels of the blob have been reset, so you won't find it again
                    log::debug!(
                        "reset found because of max contour length id={}, bearing={}, r={}, contour={}, pass={:?}",
                        target.target_id,
                        pol.bearing,
                        pol.r,
                        target.contour.length,
                        pass
                    );
                    return Err(Error::ContourLengthTooHigh);
                }

                target.lost_count = 0;
                let mut p_own = ExtendedPosition::empty();
                p_own.pos = history.spokes[history.mod_spokes(pol.raw_bearing()) as usize].pos;
                target.age_rotations += 1;
                target.status = match target.status {
                    TargetStatus::Acquire0 => TargetStatus::Acquire1,
                    TargetStatus::Acquire1 => TargetStatus::Acquire2,
                    TargetStatus::Acquire2 => TargetStatus::Acquire3,
                    TargetStatus::Acquire3 | TargetStatus::Active => TargetStatus::Active,
                    _ => TargetStatus::Acquire0,
                };
                if target.status == TargetStatus::Acquire0 {
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
                        target.target_id,
                        target.position.pos
                    );
                    target.age_rotations = 0;
                }

                // Kalman filter to  calculate the apostriori local position and speed based on found position (pol)
                if target.status == TargetStatus::Acquire2
                    || target.status == TargetStatus::Acquire3
                {
                    target.kalman.update_p();
                    target.kalman.set_measurement(
                        &mut pol,
                        &mut x_local,
                        &target.expected,
                        setup.pixels_per_meter,
                    ); // pol is measured position in polar coordinates
                }
                // x_local expected position in local coordinates

                target.position.time = pol.time; // set the target time to the newly found time, this is the time the spoke was received

                if target.status != TargetStatus::Acquire1 {
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

                if target.status == TargetStatus::Acquire2 {
                    // determine if this is a small and fast target
                    let dist_bearing = pol.raw_bearing() - bearing0;
                    let dist_r = pol.r - r0;
                    let size_bearing = max(
                        history.mod_spokes(target.contour.max_bearing - target.contour.min_bearing),
                        1,
                    );
                    let size_r = max(target.contour.max_r - target.contour.min_r, 1);
                    let test = (dist_r as f64 / size_r as f64).abs()
                        + (dist_bearing as f64 / size_bearing as f64).abs();
                    target.small_fast = test > 2.;
                    log::debug!(
                        "smallandfast, id={}, test={}, dist_r={}, size_r={}, dist_bearing={}, size_bearing={}",
                        target.target_id,
                        test,
                        dist_r,
                        size_r,
                        dist_bearing,
                        size_bearing
                    );
                }

                const FORCED_POSITION_STATUS: u32 = 8;
                const FORCED_POSITION_AGE_FAST: u32 = 5;

                if target.small_fast
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
                            "id={}, FORCED status={:?}, d_lat_dt={}, d_lon_dt={}, delta_lon_meter={}, delta_lat_meter={}, deltat={}",
                            target.target_id,
                            target.status,
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
                target.refresh_time = target.position.time;
                if target.age_rotations >= 1 {
                    let s1 = target.position.dlat_dt; // m per second
                    let s2 = target.position.dlon_dt; // m  per second
                    target.position.speed_kn = (s1 * s1 + s2 * s2).sqrt() * MS_TO_KN; // and convert to nautical miles per hour
                    target.course = f64::atan2(s2, s1).to_degrees();
                    if target.course < 0. {
                        target.course += 360.;
                    }

                    log::debug!(
                        "FOUND {:?} CYCLE id={}, status={:?}, age={}, bearing={}.{}, r={}.{}, contour={}, speed={}, sd_speed_kn={}, doppler={:?}",
                        pass,
                        target.target_id,
                        target.status,
                        target.age_rotations,
                        bearing0,
                        pol.bearing,
                        r0,
                        pol.r,
                        target.contour.length,
                        target.position.speed_kn,
                        target.position.sd_speed_kn,
                        target.doppler_target
                    );

                    target.previous_contour_length = target.contour.length;
                    // send target data to OCPN and other radar

                    const WEIGHT_FACTOR: f64 = 0.1;

                    if target.contour.length != 0 {
                        if target.average_contour_length == 0 && target.contour.length != 0 {
                            target.average_contour_length = target.contour.length;
                        } else {
                            target.average_contour_length +=
                                ((target.contour.length - target.average_contour_length) as f64
                                    * WEIGHT_FACTOR) as i32;
                        }
                    }

                    //if (status >= m_ri.m_target_age_to_mixer.GetValue()) {
                    //  f64 dist2target = pol.r / self.pixels_per_meter;
                    // TODO: PassAIVDMtoOCPN(&pol);  // status s not yet used

                    // TODO: MakeAndTransmitTargetMessage();

                    // MakeAndTransmitCoT();
                    //}

                    target.refreshed = RefreshState::Found;
                    // A target that has been found is no longer considered a transferred target
                    target.transferred_target = false;
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
        self.total_pix = 0;
        self.approaching_pix = 0;
        self.receding_pix = 0;
        for i in 0..self.contour.contour.len() {
            for radius in 0..history.spokes[0].sweep.len() {
                let pixel = history.spokes
                    [history.mod_spokes(self.contour.contour[i].raw_bearing())]
                .sweep[radius];
                let target = pixel.contains(HistoryPixel::TARGET); // above threshold bit
                if !target {
                    break;
                }
                let approaching = pixel.contains(HistoryPixel::APPROACHING); // this is Doppler approaching bit
                let receding = pixel.contains(HistoryPixel::RECEDING); // this is Doppler receding bit
                self.total_pix += target as u32;
                self.approaching_pix += approaching as u32;
                self.receding_pix += receding as u32;
            }
        }
    }

    /// Check doppler state of targets if Doppler is on
    fn state_transition(&mut self) {
        if !self.have_doppler || self.doppler_target == Doppler::AnyPlus {
            return;
        }

        let check_to_doppler = (self.total_pix as f64 * 0.85) as u32;
        let check_not_approaching = ((self.total_pix - self.approaching_pix) as f64 * 0.80) as u32;
        let check_not_receding = ((self.total_pix - self.receding_pix) as f64 * 0.80) as u32;

        let new = match self.doppler_target {
            Doppler::AnyDoppler | Doppler::Any => {
                // convert to APPROACHING or RECEDING
                if self.approaching_pix > self.receding_pix
                    && self.approaching_pix > check_to_doppler
                {
                    &Doppler::Approaching
                } else if self.receding_pix > self.approaching_pix
                    && self.receding_pix > check_to_doppler
                {
                    &Doppler::Receding
                } else if self.doppler_target == Doppler::AnyDoppler {
                    &Doppler::Any
                } else {
                    &self.doppler_target
                }
            }

            Doppler::Receding => {
                if self.receding_pix < check_not_approaching {
                    &Doppler::Any
                } else {
                    &self.doppler_target
                }
            }

            Doppler::Approaching => {
                if self.approaching_pix < check_not_receding {
                    &Doppler::Any
                } else {
                    &self.doppler_target
                }
            }
            _ => &self.doppler_target,
        };
        if *new != self.doppler_target {
            log::debug!(
                "Target {} Doppler state changed from {:?} to {:?}",
                self.target_id,
                self.doppler_target,
                new
            );
            self.doppler_target = *new;
        }
    }

    /*
        void ArpaTarget::TransferTargetToOtherRadar() {
          RadarInfo* other_radar = 0;
          LOG_ARPA(wxT("%s: TransferTargetToOtherRadar target_id=%i,"), m_ri.m_name, target_id);
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
          data.target_id = target_id;
          data.P = kalman.P;
          data.position = self.position;
          LOG_ARPA(wxT("%s: lat= %f, lon= %f, target_id=%i,"), m_ri.m_name, self.position.pos.lat, self.position.pos.lon, target_id);
          data.status = status;
          other_radar.m_arpa.InsertOrUpdateTargetFromOtherRadar(&data, false);
        }

        void ArpaTarget::SendTargetToNearbyRadar() {
          LOG_ARPA(wxT("%s: Send target to nearby radar, target_id=%i,"), m_ri.m_name, target_id);
          RadarInfo* long_radar = m_pi.GetLongRangeRadar();
          if (m_ri != long_radar) {
            return;
          }
          DynamicTargetData data;
          data.target_id = target_id;
          data.P = kalman.P;
          data.position = self.position;
          LOG_ARPA(wxT("%s: lat= %f, lon= %f, target_id=%i,"), m_ri.m_name, self.position.pos.lat, self.position.pos.lon, target_id);
          data.status = status;
          if (m_pi.m_inter_radar) {
            m_pi.m_inter_radar.SendTarget(data);
            LOG_ARPA(wxT(" %s, target data send id=%i"), m_ri.m_name, target_id);
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

      if (status == LOST) return;  // AIS has no "status lost" message
      s_Bear_Unit = wxEmptyString;   // Bearing Units  R or empty
      s_Course_Unit = wxT("T");      // Course type R; Realtive T; true
      s_Dist_Unit = wxT("N");        // Speed/Distance Unit K, N, S N= NM/h = Knots

      // f64 dist = pol.r / self.pixels_per_meter / NAUTICAL_MILE.;
      f64 bearing = pol.angle * 360. / m_ri.m_spokes;
      if (bearing < 0) bearing += 360;

      int mmsi = target_id % 1000000;
      GeoPosition radar_pos;
      m_ri.GetRadarPosition(&radar_pos);
      f64 target_lat, target_lon;

      target_lat = self.position.pos.lat;
      target_lon = self.position.pos.lon;
      wxString result = EncodeAIVDM(mmsi, self.position.speed_kn, target_lon, target_lat, course);
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
      //   LOG_ARPA(wxT(" id=%i, status == lost"), target_id);
      //   s_status = wxT("L");  // ?
      //   break;
      // }

      if (doppler_target == ANY) {
        s_status = wxT("Q");  // yellow
      } else {
        s_status = wxT("T");
      }

      f64 dist = pol.r / self.pixels_per_meter / NAUTICAL_MILE.;
      f64 bearing = pol.angle * 360. / m_ri.m_spokes;

      if (bearing < 0) bearing += 360;
      s_TargID = wxString::Format(wxT("%2i"), target_id);
      s_speed = wxString::Format(wxT("%4.2f"), self.position.speed_kn);
      s_course = wxString::Format(wxT("%3.1f"), course);
      if (automatic) {
        s_target_name = wxString::Format(wxT("ARPA%2i"), target_id);
      } else {
        s_target_name = wxString::Format(wxT("MARPA%2i"), target_id);
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
      LOG_ARPA(wxT("%s: send TTM, target=%i string=%s"), m_ri.m_name, target_id, nmea);
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
      int mmsi = target_id;
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
      course_string.Printf(wxT("\"%f\""), course);
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
  message << target_id;

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
  message << wxString::Format(wxT(",\"cog\":%4.1f"), course);

  message << wxT(",\"state\":");
  message << status;
  message << wxT(",\"lost_count\":");
  message << lost_count;

  message << wxT("}}");

  m_pi.SendToTargetMixer(message);
}
  */

    fn set_status_lost(&mut self) {
        self.contour = Contour::new();
        self.previous_contour_length = 0;
        self.lost_count = 0;
        self.kalman.reset_filter();
        self.status = TargetStatus::Lost;
        self.automatic = false;
        self.refresh_time = 0;
        self.course = 0.;
        self.stationary = 0;
        self.position.dlat_dt = 0.;
        self.position.dlon_dt = 0.;
        self.position.speed_kn = 0.;
    }

    /// Check if this target should be broadcast to clients
    /// Manual targets are broadcast immediately from Acquire0
    /// Automatic targets wait until Acquire3 or Active to avoid noise
    pub fn should_broadcast(&self) -> bool {
        if !self.automatic {
            // Manual targets: broadcast all acquiring states so user sees feedback
            matches!(
                self.status,
                TargetStatus::Acquire0
                    | TargetStatus::Acquire1
                    | TargetStatus::Acquire2
                    | TargetStatus::Acquire3
                    | TargetStatus::Active
            )
        } else {
            // Automatic targets: only broadcast once confirmed
            matches!(self.status, TargetStatus::Acquire3 | TargetStatus::Active)
        }
    }

    /// Convert internal ArpaTarget to Signal K API format for streaming
    pub fn to_api(&self, radar_position: &GeoPosition) -> ArpaTargetApi {
        // Calculate bearing and distance from radar to target
        let dlat = self.position.pos.lat - radar_position.lat;
        let dlon =
            (self.position.pos.lon - radar_position.lon) * radar_position.lat.to_radians().cos();

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
            id: self.target_id,
            status: match self.status {
                TargetStatus::Active => "tracking".to_string(),
                TargetStatus::Acquire3 => "acquiring".to_string(),
                TargetStatus::Lost => "lost".to_string(),
                // Acquire0, Acquire1, Acquire2 are early acquisition stages - not yet ready to show
                _ => "unknown".to_string(),
            },
            position: TargetPositionApi {
                bearing,
                distance: distance.round() as i32,
                latitude: Some(self.position.pos.lat),
                longitude: Some(self.position.pos.lon),
            },
            motion: TargetMotionApi {
                course,
                speed: speed_ms,
            },
            danger: TargetDangerApi {
                cpa: 0.0,  // TODO: implement CPA calculation
                tcpa: 0.0, // TODO: implement TCPA calculation
            },
            acquisition: if self.automatic {
                "auto".to_string()
            } else {
                "manual".to_string()
            },
            source_zone: if self.source_zone > 0 {
                Some(self.source_zone)
            } else {
                None
            },
        }
    }
}
