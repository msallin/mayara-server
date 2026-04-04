use std::io;
use std::net::SocketAddrV4;

use async_trait::async_trait;
use tokio::net::UdpSocket;

use super::GarminRadarType;
use crate::brand::CommandSender;
use crate::radar::settings::{ControlId, ControlValue, SharedControls};
use crate::radar::{Power, RadarError};

/// Garmin command sender
pub struct Command {
    radar_type: GarminRadarType,
    send_addr: SocketAddrV4,
    socket: Option<UdpSocket>,
}

impl Command {
    pub fn new(radar_type: GarminRadarType, send_addr: SocketAddrV4) -> Self {
        Command {
            radar_type,
            send_addr,
            socket: None,
        }
    }

    async fn ensure_socket(&mut self) -> io::Result<&UdpSocket> {
        if self.socket.is_none() {
            let socket = UdpSocket::bind("0.0.0.0:0").await?;
            socket.connect(self.send_addr).await?;
            self.socket = Some(socket);
        }
        Ok(self.socket.as_ref().unwrap())
    }

    /// Send a 9-byte command (packet_type + len=1 + u8 value)
    async fn send_9(&mut self, packet_type: u32, value: u8) -> io::Result<()> {
        let socket = self.ensure_socket().await?;
        let mut buf = [0u8; 9];
        buf[0..4].copy_from_slice(&packet_type.to_le_bytes());
        buf[4..8].copy_from_slice(&1u32.to_le_bytes());
        buf[8] = value;
        socket.send(&buf).await?;
        log::debug!(
            "Garmin {}: sent 9-byte command {:04X} value {}",
            self.radar_type,
            packet_type,
            value
        );
        Ok(())
    }

    /// Send a 10-byte command (packet_type + len=2 + u16 value)
    async fn send_10(&mut self, packet_type: u32, value: u16) -> io::Result<()> {
        let socket = self.ensure_socket().await?;
        let mut buf = [0u8; 10];
        buf[0..4].copy_from_slice(&packet_type.to_le_bytes());
        buf[4..8].copy_from_slice(&2u32.to_le_bytes());
        buf[8..10].copy_from_slice(&value.to_le_bytes());
        socket.send(&buf).await?;
        log::debug!(
            "Garmin {}: sent 10-byte command {:04X} value {}",
            self.radar_type,
            packet_type,
            value
        );
        Ok(())
    }

    /// Send a 12-byte command (packet_type + len=4 + u32 value)
    async fn send_12(&mut self, packet_type: u32, value: u32) -> io::Result<()> {
        let socket = self.ensure_socket().await?;
        let mut buf = [0u8; 12];
        buf[0..4].copy_from_slice(&packet_type.to_le_bytes());
        buf[4..8].copy_from_slice(&4u32.to_le_bytes());
        buf[8..12].copy_from_slice(&value.to_le_bytes());
        socket.send(&buf).await?;
        log::debug!(
            "Garmin {}: sent 12-byte command {:04X} value {}",
            self.radar_type,
            packet_type,
            value
        );
        Ok(())
    }

    async fn set_transmit_hd(&mut self, on: bool) -> io::Result<()> {
        // HD: 0x2b2, 1=off, 2=on
        self.send_10(0x2b2, if on { 2 } else { 1 }).await
    }

    async fn set_transmit_xhd(&mut self, on: bool) -> io::Result<()> {
        // xHD: 0x919, 0=off, 1=on
        self.send_9(0x919, if on { 1 } else { 0 }).await
    }

    async fn set_range_hd(&mut self, meters: u32) -> io::Result<()> {
        // HD: 0x2b3, meters - 1
        self.send_12(0x2b3, meters.saturating_sub(1)).await
    }

    async fn set_range_xhd(&mut self, meters: u32) -> io::Result<()> {
        // xHD: 0x91e, meters direct
        self.send_12(0x91e, meters).await
    }

    async fn set_gain_hd(&mut self, auto: bool, value: u32) -> io::Result<()> {
        // HD: 0x2b4, 0-100 manual or 344 for auto
        let level = if auto { 344 } else { value.min(100) };
        self.send_12(0x2b4, level).await
    }

    async fn set_gain_xhd(&mut self, auto: bool, auto_high: bool, value: u32) -> io::Result<()> {
        // xHD: mode via 0x924 (0=manual, 2=auto), level via 0x925 (×100), auto level via 0x91d
        if auto {
            self.send_9(0x924, 2).await?;
            self.send_9(0x91d, if auto_high { 1 } else { 0 }).await
        } else {
            self.send_9(0x924, 0).await?;
            self.send_10(0x925, (value * 100).min(10000) as u16).await
        }
    }

    async fn set_bearing_alignment_hd(&mut self, radians: f64) -> io::Result<()> {
        // HD: 0x2b7, direct degrees
        let degrees = radians.to_degrees() as i16;
        self.send_10(0x2b7, degrees as u16).await
    }

    async fn set_bearing_alignment_xhd(&mut self, radians: f64) -> io::Result<()> {
        // xHD: 0x930, degrees × 32
        let degrees = radians.to_degrees() as i32;
        self.send_12(0x930, (degrees * 32) as u32).await
    }

    async fn set_interference_hd(&mut self, value: u8) -> io::Result<()> {
        // HD: 0x2b9
        self.send_9(0x2b9, value).await
    }

    async fn set_interference_xhd(&mut self, value: u8) -> io::Result<()> {
        // xHD: send to all three packet types
        self.send_9(0x91b, value).await?;
        self.send_9(0x932, value).await?;
        self.send_9(0x2b9, value).await
    }

    async fn set_rain_hd(&mut self, value: u8) -> io::Result<()> {
        // HD: 0x2b6
        self.send_12(0x2b6, value as u32).await
    }

    async fn set_rain_xhd(&mut self, enabled: bool, value: u8) -> io::Result<()> {
        // xHD: mode via 0x933, level via 0x934
        if enabled {
            self.send_9(0x933, 1).await?;
            self.send_10(0x934, (value as u16) * 100).await
        } else {
            self.send_9(0x933, 0).await
        }
    }

    async fn set_sea_hd(&mut self, auto: bool, value: u8) -> io::Result<()> {
        // HD: 0x2b5 with mode parameter
        let socket = self.ensure_socket().await?;
        let mut buf = [0u8; 16];
        buf[0..4].copy_from_slice(&0x2b5u32.to_le_bytes());
        buf[4..8].copy_from_slice(&8u32.to_le_bytes()); // len = 8
        buf[8..12].copy_from_slice(&(value as u32).to_le_bytes());
        buf[12..16].copy_from_slice(&(if auto { 2u32 } else { 1u32 }).to_le_bytes());
        socket.send(&buf).await?;
        Ok(())
    }

    async fn set_sea_xhd(&mut self, auto: bool, auto_level: u8, value: u8) -> io::Result<()> {
        // xHD: mode via 0x939, level via 0x93a, auto level via 0x93b
        if auto {
            self.send_9(0x939, 2).await?;
            self.send_9(0x93b, auto_level).await
        } else if value == 0 {
            self.send_9(0x939, 0).await
        } else {
            self.send_9(0x939, 1).await?;
            self.send_10(0x93a, (value as u16) * 100).await
        }
    }

    async fn set_scan_speed_hd(&mut self, value: u8) -> io::Result<()> {
        // HD: 0x2be
        self.send_9(0x2be, value).await
    }

    async fn set_scan_speed_xhd(&mut self, value: u8) -> io::Result<()> {
        // xHD: 0x916, value × 2
        self.send_9(0x916, value * 2).await
    }

    async fn set_no_transmit_zone_xhd(
        &mut self,
        enabled: bool,
        start_rad: f64,
        end_rad: f64,
    ) -> io::Result<()> {
        // xHD only: 0x93f (enable), 0x940 (start), 0x941 (end)
        // Convert radians to degrees × 32
        if enabled {
            let start_deg = start_rad.to_degrees() as i32;
            let end_deg = end_rad.to_degrees() as i32;
            self.send_9(0x93f, 1).await?;
            self.send_12(0x940, (start_deg * 32) as u32).await?;
            self.send_12(0x941, (end_deg * 32) as u32).await
        } else {
            self.send_9(0x93f, 0).await
        }
    }
}

#[async_trait]
impl CommandSender for Command {
    async fn set_control(
        &mut self,
        cv: &ControlValue,
        _controls: &SharedControls,
    ) -> Result<(), RadarError> {
        let value = cv.as_i32().unwrap_or(0);
        let auto = cv.auto.unwrap_or(false);
        let enabled = cv.enabled.unwrap_or(true);
        let auto_value = cv.auto_as_f64().unwrap_or(0.0);

        log::debug!(
            "Garmin {}: set_control {:?} value={} auto={} enabled={}",
            self.radar_type,
            cv.id,
            value,
            auto,
            enabled
        );

        let result = match (&cv.id, self.radar_type) {
            (ControlId::Power, GarminRadarType::HD) => {
                let on = match Power::from_value(&cv.as_value()?).unwrap_or(Power::Standby) {
                    Power::Transmit => true,
                    _ => false,
                };
                self.set_transmit_hd(on).await
            }
            (ControlId::Power, GarminRadarType::XHD) => {
                let on = match Power::from_value(&cv.as_value()?).unwrap_or(Power::Standby) {
                    Power::Transmit => true,
                    _ => false,
                };
                self.set_transmit_xhd(on).await
            }
            (ControlId::Range, GarminRadarType::HD) => self.set_range_hd(value as u32).await,
            (ControlId::Range, GarminRadarType::XHD) => self.set_range_xhd(value as u32).await,
            (ControlId::Gain, GarminRadarType::HD) => self.set_gain_hd(auto, value as u32).await,
            (ControlId::Gain, GarminRadarType::XHD) => {
                // auto_value 0 = low, 1 = high
                let auto_high = auto_value > 0.5;
                self.set_gain_xhd(auto, auto_high, value as u32).await
            }
            (ControlId::BearingAlignment, GarminRadarType::HD) => {
                let radians = cv.as_f64().unwrap_or(0.0);
                self.set_bearing_alignment_hd(radians).await
            }
            (ControlId::BearingAlignment, GarminRadarType::XHD) => {
                let radians = cv.as_f64().unwrap_or(0.0);
                self.set_bearing_alignment_xhd(radians).await
            }
            (ControlId::InterferenceRejection, GarminRadarType::HD) => {
                self.set_interference_hd(value as u8).await
            }
            (ControlId::InterferenceRejection, GarminRadarType::XHD) => {
                self.set_interference_xhd(value as u8).await
            }
            (ControlId::Rain, GarminRadarType::HD) => self.set_rain_hd(value as u8).await,
            (ControlId::Rain, GarminRadarType::XHD) => {
                self.set_rain_xhd(enabled, value as u8).await
            }
            (ControlId::Sea, GarminRadarType::HD) => self.set_sea_hd(auto, value as u8).await,
            (ControlId::Sea, GarminRadarType::XHD) => {
                let auto_level = auto_value as u8;
                self.set_sea_xhd(auto, auto_level, value as u8).await
            }
            (ControlId::ScanSpeed, GarminRadarType::HD) => {
                self.set_scan_speed_hd(value as u8).await
            }
            (ControlId::ScanSpeed, GarminRadarType::XHD) => {
                self.set_scan_speed_xhd(value as u8).await
            }
            (ControlId::NoTransmitSector1, GarminRadarType::XHD) => {
                let start_rad = cv.as_f64().unwrap_or(0.0);
                let end_rad = cv.end_as_f64().unwrap_or(0.0);
                self.set_no_transmit_zone_xhd(enabled, start_rad, end_rad)
                    .await
            }
            _ => {
                log::debug!("Garmin {}: unhandled control {:?}", self.radar_type, cv.id);
                Ok(())
            }
        };

        result.map_err(RadarError::Io)
    }
}
