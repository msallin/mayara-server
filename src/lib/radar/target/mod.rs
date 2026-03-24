//! Target tracking API types and utilities.
//!
//! This module provides the API types for representing tracked targets
//! that are sent to the GUI via Signal K, as well as blob detection for
//! identifying potential targets, and target tracking with Kalman filtering.

mod blob;
mod kalman;
mod manager;
mod motion;
mod tracker;

pub use blob::{BlobDetector, CompletedBlob, MAX_TARGET_SIZE_M, MIN_TARGET_SIZE_M};
pub use manager::{BlobMessage, MarpaRequest, SpokeContext, TrackerManager};
pub use motion::{MotionModel, TrackingMode, create_motion_model};
pub use tracker::{ActiveTarget, CandidateSource, ProcessResult, TargetCandidate, TargetStatus, TargetTracker};

use serde::Serialize;
use utoipa::ToSchema;

use super::NAUTICAL_MILE_F64;

// ============================================================================
// Geographic constants and utilities
// ============================================================================

pub const METERS_PER_DEGREE_LATITUDE: f64 = 60. * NAUTICAL_MILE_F64;
pub const KN_TO_MS: f64 = NAUTICAL_MILE_F64 / 3600.;
pub const MS_TO_KN: f64 = 3600. / NAUTICAL_MILE_F64;

/// The length of a degree longitude varies by the latitude,
/// the more north or south you get the shorter it becomes.
/// Since the earth is _nearly_ a sphere, the cosine function
/// is _very_ close.
pub fn meters_per_degree_longitude(lat: &f64) -> f64 {
    METERS_PER_DEGREE_LATITUDE * lat.to_radians().cos()
}

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
    /// Collision danger assessment - omitted if vessels diverging
    #[serde(skip_serializing_if = "TargetDangerApi::is_empty")]
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

/// Collision danger assessment in the API format.
/// Entire field is omitted when vessels are diverging (no CPA).
#[derive(Serialize, Clone, Debug, ToSchema)]
pub struct TargetDangerApi {
    /// Closest Point of Approach in meters
    pub cpa: f64,
    /// Time to CPA in seconds
    pub tcpa: f64,
}

impl TargetDangerApi {
    fn is_empty(&self) -> bool {
        self.cpa == 0.0 && self.tcpa == 0.0
    }
}
