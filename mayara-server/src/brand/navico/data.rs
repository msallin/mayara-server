use std::time::{SystemTime, UNIX_EPOCH};
use std::{io, time::Duration};
use tokio::net::UdpSocket;
use tokio::time::sleep;
use tokio_graceful_shutdown::SubsystemHandle;
use trail::TrailBuffer;

// Use mayara-core for spoke parsing (pure, WASM-compatible)
use mayara_core::protocol::navico::{
    parse_4g_spoke_header, parse_br24_spoke_header, DopplerMode, SpokeProcessor, SPOKE_HEADER_SIZE,
};

use crate::locator::LocatorId;
use crate::network::create_udp_multicast_listen;
use crate::protos::RadarMessage::RadarMessage;
use crate::radar::spoke::{to_protobuf_spoke, GenericSpoke};
use crate::settings::DataUpdate;
use crate::util::PrintableSpoke;
use crate::{radar::*, Session};

use super::{NAVICO_SPOKES, RADAR_LINE_DATA_LENGTH, SPOKES_PER_FRAME};

const FRAME_HEADER_LENGTH: usize = 8;
const RADAR_LINE_HEADER_LENGTH: usize = SPOKE_HEADER_SIZE;
const RADAR_LINE_LENGTH: usize = SPOKE_HEADER_SIZE + RADAR_LINE_DATA_LENGTH;
// Buffer size for UDP frame: header + 32 spokes
const RADAR_FRAME_BUFFER_SIZE: usize = FRAME_HEADER_LENGTH + (SPOKES_PER_FRAME * RADAR_LINE_LENGTH);

pub struct NavicoDataReceiver {
    key: String,
    statistics: Statistics,
    info: RadarInfo,
    sock: Option<UdpSocket>,
    data_update_rx: tokio::sync::broadcast::Receiver<DataUpdate>,
    doppler: DopplerMode,
    spoke_processor: SpokeProcessor,
    trails: TrailBuffer,
    prev_angle: u16,
    replay: bool,
}

impl NavicoDataReceiver {
    pub fn new(session: &Session, info: RadarInfo) -> NavicoDataReceiver {
        let key = info.key();

        let data_update_rx = info.controls.data_update_subscribe();

        let spoke_processor = SpokeProcessor::new(
            info.legend.doppler_approaching,
            info.legend.doppler_receding,
        );
        let trails = TrailBuffer::new(session.clone(), &info);
        let replay = session.read().unwrap().args.replay;

        log::debug!(
            "{}: Creating NavicoDataReceiver with SpokeProcessor",
            key,
        );

        NavicoDataReceiver {
            key,
            statistics: Statistics::new(),
            info,
            sock: None,
            data_update_rx,
            doppler: DopplerMode::None,
            spoke_processor,
            trails,
            prev_angle: 0,
            replay,
        }
    }

    async fn start_socket(&mut self) -> io::Result<()> {
        match create_udp_multicast_listen(&self.info.spoke_data_addr, &self.info.nic_addr) {
            Ok(sock) => {
                self.sock = Some(sock);
                log::debug!(
                    "{} via {}: listening for spoke data",
                    &self.info.spoke_data_addr,
                    &self.info.nic_addr
                );
                Ok(())
            }
            Err(e) => {
                sleep(Duration::from_millis(1000)).await;
                log::debug!(
                    "{} via {}: create multicast failed: {}",
                    &self.info.spoke_data_addr,
                    &self.info.nic_addr,
                    e
                );
                Ok(())
            }
        }
    }

    async fn handle_data_update(&mut self, r: DataUpdate) -> Result<(), RadarError> {
        log::debug!("{}: Received data update: {:?}", self.key, r);
        match r {
            DataUpdate::Doppler(doppler) => {
                // Convert server DopplerMode to core DopplerMode (same enum values)
                self.doppler = match doppler {
                    crate::radar::DopplerMode::None => DopplerMode::None,
                    crate::radar::DopplerMode::Both => DopplerMode::Both,
                    crate::radar::DopplerMode::Approaching => DopplerMode::Approaching,
                };
            }
            DataUpdate::Legend(legend) => {
                // Rebuild spoke processor with new Doppler indices
                self.spoke_processor = SpokeProcessor::new(
                    legend.doppler_approaching,
                    legend.doppler_receding,
                );
                self.info.legend = legend;
            }
            DataUpdate::Ranges(_) => {
                // Navico DataReceiver does not need to know what ranges are in use.
            }
            DataUpdate::ControlValue(reply_tx, cv) => {
                match self.trails.set_control_value(&self.info.controls, &cv) {
                    Ok(()) => {
                        return Ok(());
                    }
                    Err(e) => {
                        return self
                            .info
                            .controls
                            .send_error_to_client(reply_tx, &cv, &e)
                            .await;
                    }
                };
            }
        }

        Ok(())
    }

    pub async fn run(mut self, subsys: SubsystemHandle) -> Result<(), RadarError> {
        self.start_socket().await.unwrap();
        loop {
            if self.sock.is_some() {
                match self.socket_loop(&subsys).await {
                    Err(RadarError::Shutdown) => {
                        return Ok(());
                    }
                    _ => {
                        // Ignore, reopen socket
                    }
                }
                self.sock = None;
            } else {
                sleep(Duration::from_millis(1000)).await;
                self.start_socket().await.unwrap();
            }
        }
    }

    async fn socket_loop(&mut self, subsys: &SubsystemHandle) -> Result<(), RadarError> {
        let mut buf = Vec::with_capacity(RADAR_FRAME_BUFFER_SIZE);
        log::trace!(
            "{}: Starting socket loop on {}",
            self.key,
            self.info.spoke_data_addr
        );

        loop {
            tokio::select! {
                _ = subsys.on_shutdown_requested() => {
                    return Err(RadarError::Shutdown);
                },
                r = self.data_update_rx.recv() => {
                    match r {
                        Ok(data_update) => {
                            self.handle_data_update(data_update).await?;
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            log::warn!("{}: data_update receiver lagged, skipped {} messages", self.key, n);
                            // Continue - don't crash, just log the dropped messages
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                            log::error!("{}: data_update channel closed unexpectedly", self.key);
                            return Err(RadarError::Shutdown);
                        }
                    }
                },
                r = self.sock.as_ref().unwrap().recv_buf_from(&mut buf)  => {
                    match r {
                        Ok(_) => {
                            self.process_frame(&mut buf);
                        },
                        Err(e) => {
                            return Err(RadarError::Io(e));
                        }
                    }
                },
            }
            buf.clear();
        }
    }

    fn process_frame(&mut self, data: &mut Vec<u8>) {
        if data.len() < FRAME_HEADER_LENGTH + RADAR_LINE_LENGTH {
            log::warn!(
                "UDP data frame with even less than one spoke, len {} dropped",
                data.len()
            );
            return;
        }

        let mut spokes_in_frame = (data.len() - FRAME_HEADER_LENGTH) / RADAR_LINE_LENGTH;
        if spokes_in_frame != 32 {
            self.statistics.broken_packets += 1;
            if spokes_in_frame > 32 {
                spokes_in_frame = 32;
            }
        }

        log::trace!("Received UDP frame with {} spokes", &spokes_in_frame);

        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .ok();

        let mut mark_full_rotation = false;
        let mut message = RadarMessage::new();
        message.radar = self.info.id as u32;

        let mut offset: usize = FRAME_HEADER_LENGTH;
        for scanline in 0..spokes_in_frame {
            let header_slice = &data[offset..offset + RADAR_LINE_HEADER_LENGTH];
            let spoke_slice = &data[offset + RADAR_LINE_HEADER_LENGTH
                ..offset + RADAR_LINE_HEADER_LENGTH + RADAR_LINE_DATA_LENGTH];

            if let Some((range, angle, heading)) = self.validate_header(header_slice, scanline) {
                log::trace!("range {} angle {} heading {:?}", range, angle, heading);
                log::trace!(
                    "Received {:04} spoke {}",
                    scanline,
                    PrintableSpoke::new(spoke_slice)
                );
                let mut spoke = to_protobuf_spoke(
                    &self.info,
                    range,
                    angle,
                    heading,
                    now,
                    self.process_spoke(spoke_slice),
                );
                self.trails.update_trails(&mut spoke, &self.info.legend);
                message.spokes.push(spoke);

                if angle < self.prev_angle {
                    mark_full_rotation = true;
                }
                if ((self.prev_angle + 1) % NAVICO_SPOKES as u16) != angle {
                    self.statistics.missing_spokes +=
                        (angle + NAVICO_SPOKES as u16 - self.prev_angle - 1) as usize
                            % NAVICO_SPOKES as usize;
                    log::trace!("{}: Spoke angle {} is not consecutive to previous angle {}, new missing spokes {}",
                        self.key, angle, self.prev_angle, self.statistics.missing_spokes);
                }
                self.statistics.received_spokes += 1;
                self.prev_angle = angle;
            } else {
                log::warn!("Invalid spoke: header {:02X?}", &header_slice);
                self.statistics.broken_packets += 1;
            }

            offset += RADAR_LINE_LENGTH;
        }

        if mark_full_rotation {
            let ms = self.info.full_rotation();
            self.trails.set_rotation_speed(ms);
            self.statistics.full_rotation(&self.key);
        }

        self.info.broadcast_radar_message(message);
    }

    fn validate_header(
        &self,
        header_slice: &[u8],
        scanline: usize,
    ) -> Option<(u32, SpokeBearing, Option<u16>)> {
        // Use core parsing functions
        let result = match self.info.locator_id {
            LocatorId::Gen3Plus => parse_4g_spoke_header(header_slice),
            LocatorId::GenBR24 => parse_br24_spoke_header(header_slice),
            _ => {
                panic!("Incorrect Navico type");
            }
        };

        match result {
            Ok((range, angle, heading)) => {
                log::trace!(
                    "Received {:04} spoke: range={} angle={} heading={:?}",
                    scanline,
                    range,
                    angle,
                    heading
                );
                Some((range, angle, heading))
            }
            Err(e) => {
                log::warn!("Invalid spoke header: {} data {:02X?}", e, &header_slice);
                None
            }
        }
    }

    fn process_spoke(&self, spoke: &[u8]) -> GenericSpoke {
        // Use core's SpokeProcessor for pure data transformation
        let mut generic_spoke = self.spoke_processor.process_spoke(spoke, self.doppler);

        if self.replay {
            // Generate circle at extreme range (for replay visualization)
            let last_two = self
                .spoke_processor
                .process_spoke(&[0xff], self.doppler);
            let len = generic_spoke.len();
            if len >= 2 {
                generic_spoke[len - 2] = last_two[0];
                generic_spoke[len - 1] = last_two[1];
            }
        }

        generic_spoke
    }
}
