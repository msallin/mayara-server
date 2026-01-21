//! Radar data structures
//!
//! These structures represent radar metadata and configuration,
//! independent of any I/O or networking code.

use crate::Brand;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::net::{Ipv4Addr, SocketAddrV4};

// =============================================================================
// Serde helpers for SocketAddrV4/Ipv4Addr <-> String
// =============================================================================

mod socket_addr_serde {
    use super::*;

    pub fn serialize<S: Serializer>(addr: &SocketAddrV4, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&addr.to_string())
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<SocketAddrV4, D::Error> {
        let s = String::deserialize(d)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

mod option_socket_addr_serde {
    use super::*;

    pub fn serialize<S: Serializer>(addr: &Option<SocketAddrV4>, s: S) -> Result<S::Ok, S::Error> {
        match addr {
            Some(a) => s.serialize_some(&a.to_string()),
            None => s.serialize_none(),
        }
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Option<SocketAddrV4>, D::Error> {
        let opt: Option<String> = Option::deserialize(d)?;
        match opt {
            Some(s) => s.parse().map(Some).map_err(serde::de::Error::custom),
            None => Ok(None),
        }
    }
}

mod option_ipv4_serde {
    use super::*;

    pub fn serialize<S: Serializer>(addr: &Option<Ipv4Addr>, s: S) -> Result<S::Ok, S::Error> {
        match addr {
            Some(a) => s.serialize_some(&a.to_string()),
            None => s.serialize_none(),
        }
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Option<Ipv4Addr>, D::Error> {
        let opt: Option<String> = Option::deserialize(d)?;
        match opt {
            Some(s) => s.parse().map(Some).map_err(serde::de::Error::custom),
            None => Ok(None),
        }
    }
}

/// Basic radar information discovered from beacon response
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RadarDiscovery {
    /// Radar brand
    pub brand: Brand,
    /// Radar model (if known)
    pub model: Option<String>,
    /// Radar name/serial from beacon
    pub name: String,
    /// Primary radar address (IP + port)
    #[serde(with = "socket_addr_serde")]
    pub address: SocketAddrV4,
    /// Number of spokes per revolution
    pub spokes_per_revolution: u16,
    /// Maximum spoke length in pixels
    pub max_spoke_len: u16,
    /// Pixel depth (e.g., 16, 64, 128)
    pub pixel_values: u8,
    /// Serial number (from model report)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub serial_number: Option<String>,
    /// NIC address that received this beacon (for multi-interface systems)
    #[serde(skip_serializing_if = "Option::is_none", with = "option_ipv4_serde")]
    pub nic_address: Option<Ipv4Addr>,
    /// Suffix for dual-range radars ("A" or "B"), None for single-range
    #[serde(skip_serializing_if = "Option::is_none")]
    pub suffix: Option<String>,
    /// Data streaming address (for brands like Navico that use separate multicast addresses)
    #[serde(skip_serializing_if = "Option::is_none", with = "option_socket_addr_serde")]
    pub data_address: Option<SocketAddrV4>,
    /// Report/status address (for brands like Navico that use separate multicast addresses)
    #[serde(skip_serializing_if = "Option::is_none", with = "option_socket_addr_serde")]
    pub report_address: Option<SocketAddrV4>,
    /// Send/command address
    #[serde(skip_serializing_if = "Option::is_none", with = "option_socket_addr_serde")]
    pub send_address: Option<SocketAddrV4>,
}

/// Legend entry for mapping pixel values to colors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegendEntry {
    /// Pixel type (Normal, TargetBorder, DopplerApproaching, etc.)
    #[serde(rename = "type")]
    pub pixel_type: String,
    /// RGBA color as hex string (e.g., "#00FF00FF")
    pub color: String,
}

/// Radar control value
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ControlValue {
    /// Current value
    pub value: serde_json::Value,
    /// Whether this control is enabled
    #[serde(default)]
    pub enabled: bool,
    /// Whether this control is in auto mode
    #[serde(default)]
    pub auto: bool,
}

/// Radar control definition
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ControlDefinition {
    /// Control name
    pub name: String,
    /// Control description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Minimum value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    /// Maximum value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
    /// Step value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step: Option<f64>,
    /// Unit (e.g., "meters", "degrees")
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    /// Whether this control supports auto mode
    #[serde(default)]
    pub has_auto: bool,
}

/// Full radar state including controls and legend
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RadarState {
    /// Radar ID
    pub id: String,
    /// Radar name
    pub name: String,
    /// Brand
    pub brand: Brand,
    /// Model (if known)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    /// Current status
    pub status: RadarStatus,
    /// Number of spokes per revolution
    pub spokes_per_revolution: u16,
    /// Maximum spoke length
    pub max_spoke_len: u16,
    /// Legend for pixel color mapping
    pub legend: Vec<LegendEntry>,
    /// Available controls with current values
    pub controls: std::collections::HashMap<String, ControlValue>,
    /// Optional external stream URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stream_url: Option<String>,
}

/// Radar operational status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RadarStatus {
    /// Radar is off
    Off,
    /// Radar is warming up
    Warming,
    /// Radar is in standby mode
    Standby,
    /// Radar is transmitting
    Transmit,
    /// Radar status unknown
    Unknown,
}

impl Default for RadarStatus {
    fn default() -> Self {
        RadarStatus::Unknown
    }
}

impl std::fmt::Display for RadarStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadarStatus::Off => write!(f, "off"),
            RadarStatus::Warming => write!(f, "warming"),
            RadarStatus::Standby => write!(f, "standby"),
            RadarStatus::Transmit => write!(f, "transmit"),
            RadarStatus::Unknown => write!(f, "unknown"),
        }
    }
}

// =============================================================================
// Palette Generation
// =============================================================================

/// RGBA color for palette generation
#[derive(Debug, Clone, Copy)]
pub struct Rgba {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Rgba {
    pub const fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self { r, g, b, a }
    }

    /// Convert to hex string "#RRGGBBAA"
    pub fn to_hex(&self) -> String {
        format!("#{:02x}{:02x}{:02x}{:02x}", self.r, self.g, self.b, self.a)
    }
}

/// Generate color palette for radar display based on pixel value count.
///
/// Creates a smooth color gradient: Blue → Cyan → Green → Yellow → Red
/// This is the core palette algorithm used by both server and GUI.
///
/// # Arguments
/// * `pixel_values` - Number of distinct intensity values (e.g., 16 for Navico, 64 for Furuno)
///
/// # Returns
/// Vector of RGBA colors, where index 0 is transparent (noise floor)
pub fn generate_palette(pixel_values: u8) -> Vec<Rgba> {
    const MIN_INTENSITY: f64 = 85.0; // Start at 1/3 intensity for visibility
    const MAX_INTENSITY: f64 = 255.0;

    let mut palette = Vec::with_capacity(pixel_values as usize);

    // Clamp pixel_values to valid range
    let pixel_values = pixel_values.min(255 - 32 - 2);
    if pixel_values == 0 {
        return palette;
    }

    let pixels_with_color = pixel_values.saturating_sub(1);

    // Index 0: transparent/black (noise floor)
    palette.push(Rgba::new(0, 0, 0, 0));

    // Generate color gradient for indices 1 to pixel_values-1
    for v in 1..pixel_values {
        // Normalize v to 0.0 - 1.0 range
        let t = (v - 1) as f64 / pixels_with_color.max(1) as f64;

        // Color progression: Blue → Cyan → Green → Yellow → Red
        let (r, g, b) = if t < 0.25 {
            // Blue to Cyan: increase green
            let local_t = t / 0.25;
            (0.0, local_t, 1.0)
        } else if t < 0.5 {
            // Cyan to Green: decrease blue
            let local_t = (t - 0.25) / 0.25;
            (0.0, 1.0, 1.0 - local_t)
        } else if t < 0.75 {
            // Green to Yellow: increase red
            let local_t = (t - 0.5) / 0.25;
            (local_t, 1.0, 0.0)
        } else {
            // Yellow to Red: decrease green
            let local_t = (t - 0.75) / 0.25;
            (1.0, 1.0 - local_t, 0.0)
        };

        // Apply intensity scaling
        let scale = |c: f64| -> u8 {
            if c > 0.0 {
                (MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * c) as u8
            } else {
                0
            }
        };

        palette.push(Rgba::new(scale(r), scale(g), scale(b), 255));
    }

    palette
}

/// Generate legend entries from a palette.
///
/// This converts the raw palette colors to the LegendEntry format
/// expected by the radar API.
pub fn generate_legend(pixel_values: u8) -> Vec<LegendEntry> {
    generate_palette(pixel_values)
        .into_iter()
        .enumerate()
        .map(|(i, rgba)| LegendEntry {
            pixel_type: format!("level_{}", i),
            color: rgba.to_hex(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_palette_generation() {
        // Test with Navico's 16 values
        let palette = generate_palette(16);
        assert_eq!(palette.len(), 16);
        assert_eq!(palette[0].a, 0); // First entry is transparent
        assert_eq!(palette[1].a, 255); // Others are opaque
    }

    #[test]
    fn test_legend_generation() {
        let legend = generate_legend(16);
        assert_eq!(legend.len(), 16);
        assert!(legend[0].color.starts_with("#"));
        assert_eq!(legend[0].pixel_type, "level_0");
    }

    #[test]
    fn test_rgba_to_hex() {
        let rgba = Rgba::new(255, 128, 0, 255);
        assert_eq!(rgba.to_hex(), "#ff8000ff");
    }
}
