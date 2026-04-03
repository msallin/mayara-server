use std::collections::HashMap;
use std::fmt::{self, Display};
use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4};
use tokio_graceful_shutdown::{SubsystemBuilder, SubsystemHandle};

use crate::brand::{LocatorId, RadarLocator};
use crate::locator::LocatorAddress;
use crate::radar::range::Ranges;
use crate::radar::{RadarInfo, SharedRadars};
use crate::{Brand, Cli};

mod command;
mod report;
mod settings;

// Garmin radars use 172.16.x.x subnet
// We match using 172.16.0.0/12 (netmask 255.240.0.0) like Furuno does for 172.31.x.x
const GARMIN_BEACON_ADDRESS: SocketAddr =
    SocketAddr::new(IpAddr::V4(Ipv4Addr::new(239, 254, 2, 0)), 50100);

const GARMIN_SEND_PORT: u16 = 50101;

// HD: 720 spokes per revolution, 0.5° resolution
const GARMIN_HD_SPOKES: usize = 720;
// xHD: 1440 spokes per revolution, 0.25° resolution
const GARMIN_XHD_SPOKES: usize = 1440;

// HD: 1-bit samples, up to 2016 samples per spoke (252 bytes × 8)
const GARMIN_HD_SPOKE_LEN: usize = 2016;
// xHD: 8-bit samples, up to ~700 samples per spoke
const GARMIN_XHD_SPOKE_LEN: usize = 705;

// HD has 1-bit binary data, we convert to 0 or 255
const HD_PIXEL_VALUES: u8 = 2;
// xHD has 8-bit data but we halve it (like Raymarine) to make room for legend values
const XHD_PIXEL_VALUES_RAW: u16 = 256;
const XHD_PIXEL_VALUES: u8 = (XHD_PIXEL_VALUES_RAW / 2) as u8;

// 1 nautical mile in meters
const NM: i32 = 1852;

// Garmin HD metric ranges (meters)
const GARMIN_HD_RANGES_METRIC: &[i32] = &[
    250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000, 16000, 24000, 36000, 48000,
    64000,
];

// Garmin HD nautical ranges (meters, based on NM fractions)
const GARMIN_HD_RANGES_NAUTICAL: &[i32] = &[
    232,        // ~1/8 NM
    NM / 4,     // 463
    NM / 2,     // 926
    NM * 3 / 4, // 1389
    NM,         // 1852
    NM * 3 / 2, // 2778
    NM * 2,     // 3704
    NM * 3,     // 5556
    NM * 4,     // 7408
    NM * 6,     // 11112
    NM * 8,     // 14816
    NM * 12,    // 22224
    NM * 16,    // 29632
    NM * 24,    // 44448
    NM * 36,    // 66672
    NM * 48,    // 88896
];

// Garmin xHD metric ranges (meters) - same as HD
const GARMIN_XHD_RANGES_METRIC: &[i32] = GARMIN_HD_RANGES_METRIC;

// Garmin xHD nautical ranges (meters, based on NM fractions)
// xHD starts at 1/8 NM instead of 232
const GARMIN_XHD_RANGES_NAUTICAL: &[i32] = &[
    NM / 8,     // 232 (1/8 NM)
    NM / 4,     // 463
    NM / 2,     // 926
    NM * 3 / 4, // 1389
    NM,         // 1852
    NM * 3 / 2, // 2778
    NM * 2,     // 3704
    NM * 3,     // 5556
    NM * 4,     // 7408
    NM * 6,     // 11112
    NM * 8,     // 14816
    NM * 12,    // 22224
    NM * 16,    // 29632
    NM * 24,    // 44448
    NM * 36,    // 66672
    NM * 48,    // 88896
];

/// Get supported ranges for a Garmin radar type
fn get_ranges(radar_type: GarminRadarType) -> Ranges {
    let (metric, nautical) = match radar_type {
        GarminRadarType::HD => (GARMIN_HD_RANGES_METRIC, GARMIN_HD_RANGES_NAUTICAL),
        _ => (GARMIN_XHD_RANGES_METRIC, GARMIN_XHD_RANGES_NAUTICAL),
    };

    // Combine both metric and nautical ranges
    let mut all: Vec<i32> = metric.to_vec();
    for &r in nautical {
        if !all.contains(&r) {
            all.push(r);
        }
    }

    Ranges::new_by_distance(&all)
}

/// Supported Garmin radar types
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum GarminRadarType {
    /// Original HD radar: 720 spokes, 1-bit samples
    HD,
    /// xHD radar: 1440 spokes, 8-bit samples
    XHD,
    /// xHD2 radar: NOT YET SUPPORTED (different protocol)
    XHD2,
    /// xHD3 radar: NOT YET SUPPORTED (different protocol)
    XHD3,
    /// Fantom radar: NOT YET SUPPORTED (different protocol)
    Fantom,
}

impl Display for GarminRadarType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s: &'static str = match self {
            GarminRadarType::HD => "HD",
            GarminRadarType::XHD => "xHD",
            GarminRadarType::XHD2 => "xHD2",
            GarminRadarType::XHD3 => "xHD3",
            GarminRadarType::Fantom => "Fantom",
        };
        write!(f, "{}", s)
    }
}

impl GarminRadarType {
    /// Returns the number of spokes per revolution for this radar type
    pub fn spokes_per_revolution(&self) -> usize {
        match self {
            GarminRadarType::HD => GARMIN_HD_SPOKES,
            GarminRadarType::XHD => GARMIN_XHD_SPOKES,
            // Unsupported types default to xHD specs
            _ => GARMIN_XHD_SPOKES,
        }
    }

    /// Returns the maximum spoke length for this radar type
    pub fn max_spoke_len(&self) -> usize {
        match self {
            GarminRadarType::HD => GARMIN_HD_SPOKE_LEN,
            GarminRadarType::XHD => GARMIN_XHD_SPOKE_LEN,
            _ => GARMIN_XHD_SPOKE_LEN,
        }
    }

    /// Returns the number of pixel values for this radar type
    pub fn pixel_values(&self) -> u8 {
        match self {
            GarminRadarType::HD => HD_PIXEL_VALUES,
            GarminRadarType::XHD => XHD_PIXEL_VALUES,
            _ => XHD_PIXEL_VALUES,
        }
    }

    /// Returns true if this radar type is currently supported
    pub fn is_supported(&self) -> bool {
        matches!(self, GarminRadarType::HD | GarminRadarType::XHD)
    }
}

/// State for tracking radar info before we know the type
#[derive(Clone)]
struct RadarState {
    radar_type: Option<GarminRadarType>,
}

#[derive(Clone)]
struct GarminLocator {
    args: Cli,
    /// Map from radar address to state
    radars: HashMap<SocketAddrV4, RadarState>,
}

impl GarminLocator {
    fn new(args: Cli) -> Self {
        GarminLocator {
            args,
            radars: HashMap::new(),
        }
    }

    fn found(&self, info: RadarInfo, radars: &SharedRadars, subsys: &SubsystemHandle) {
        if let Some(mut info) = radars.add(info) {
            info.start_forwarding_radar_messages_to_stdout(&subsys);

            let report_name = info.key();
            radars.update(&mut info);

            let report_receiver =
                report::GarminReportReceiver::new(&self.args, info, radars.clone());

            subsys.start(SubsystemBuilder::new(report_name, |s| {
                report_receiver.run(s)
            }));
        }
    }

    /// Detect radar type from packet type
    fn detect_radar_type(packet_type: u32) -> Option<GarminRadarType> {
        match packet_type {
            // HD packet types
            0x2a3 | 0x2a5 | 0x2a7 => Some(GarminRadarType::HD),
            // xHD packet types
            0x0916 | 0x0919 | 0x091e | 0x0924 | 0x0925 | 0x091d | 0x0930 | 0x0932 | 0x0933
            | 0x0934 | 0x0939 | 0x093a | 0x093b | 0x093f | 0x0940 | 0x0941 | 0x0942 | 0x0943
            | 0x0944 | 0x0992 | 0x0993 | 0x099b => Some(GarminRadarType::XHD),
            _ => None,
        }
    }

    fn process_report(
        &mut self,
        report: &[u8],
        from: &SocketAddrV4,
        nic_addr: &Ipv4Addr,
        radars: &SharedRadars,
        subsys: &SubsystemHandle,
    ) -> io::Result<()> {
        if report.len() < 8 {
            return Ok(());
        }

        let packet_type = u32::from_le_bytes(report[0..4].try_into().unwrap());

        // Try to detect radar type from packet
        if let Some(detected_type) = Self::detect_radar_type(packet_type) {
            if !detected_type.is_supported() {
                log::warn!(
                    "{}: Detected unsupported Garmin radar type: {}",
                    from,
                    detected_type
                );
                return Ok(());
            }

            // Check if we already know this radar
            if let Some(state) = self.radars.get(from) {
                if state.radar_type.is_some() {
                    // Already registered, just process the report
                    return Ok(());
                }
            }

            log::info!(
                "{}: Detected Garmin {} radar via {}",
                from,
                detected_type,
                nic_addr
            );

            // Create radar info for this radar
            let radar_send = SocketAddrV4::new(*from.ip(), GARMIN_SEND_PORT);

            let spoke_data_addr = match detected_type {
                GarminRadarType::HD => SocketAddrV4::new(Ipv4Addr::new(239, 254, 2, 0), 50100),
                GarminRadarType::XHD => SocketAddrV4::new(Ipv4Addr::new(239, 254, 2, 0), 50102),
                _ => return Ok(()),
            };

            let report_addr = SocketAddrV4::new(Ipv4Addr::new(239, 254, 2, 0), 50100);

            let mut radar_info = RadarInfo::new(
                radars,
                &self.args,
                Brand::Garmin,
                None, // No serial number discovery yet
                None, // No A/B dual radar
                detected_type.pixel_values(),
                detected_type.spokes_per_revolution(),
                detected_type.max_spoke_len(),
                *from,
                nic_addr.clone(),
                spoke_data_addr,
                report_addr,
                radar_send,
                |id, tx| settings::new(id, tx, &self.args, detected_type),
                false, // No Doppler support
                false,
            );

            radar_info
                .controls
                .set_model_name(format!("Garmin {}", detected_type));
            radar_info
                .controls
                .set_user_name(format!("Garmin {}", detected_type));
            radar_info.set_ranges(get_ranges(detected_type));

            // Store state
            self.radars.insert(
                *from,
                RadarState {
                    radar_type: Some(detected_type),
                },
            );

            self.found(radar_info, radars, subsys);
        }

        Ok(())
    }
}

impl RadarLocator for GarminLocator {
    fn process(
        &mut self,
        message: &[u8],
        from: &SocketAddrV4,
        nic_addr: &Ipv4Addr,
        radars: &SharedRadars,
        subsys: &SubsystemHandle,
    ) -> Result<(), io::Error> {
        self.process_report(message, from, nic_addr, radars, subsys)
    }

    fn clone(&self) -> Box<dyn RadarLocator> {
        Box::new(Clone::clone(self))
    }
}

pub(super) fn new(args: &Cli, addresses: &mut Vec<LocatorAddress>) {
    // Only add Garmin locator once
    if !addresses.iter().any(|i| i.id == LocatorId::Garmin) {
        addresses.push(LocatorAddress::new(
            LocatorId::Garmin,
            &GARMIN_BEACON_ADDRESS,
            Brand::Garmin,
            vec![], // Garmin doesn't need beacon request packets
            Box::new(GarminLocator::new(args.clone())),
        ));
    }
}
