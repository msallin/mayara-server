use async_trait::async_trait;

use crate::brand::CommandSender;
use crate::radar::settings::{ControlValue, SharedControls};
use crate::radar::{RadarError, RadarInfo};

/// Command sender for the emulator radar.
/// Since this is a simulated radar, most commands are just logged.
pub struct Command {
    key: String,
}

impl Command {
    pub fn new(info: RadarInfo) -> Self {
        Command { key: info.key() }
    }
}

#[async_trait]
impl CommandSender for Command {
    async fn set_control(
        &mut self,
        cv: &ControlValue,
        _controls: &SharedControls,
    ) -> Result<(), RadarError> {
        log::debug!(
            "{}: Emulator set_control {:?} = {:?}",
            self.key,
            cv.id,
            cv.value
        );
        // Emulator just acknowledges the command - actual state is managed in report.rs
        Ok(())
    }
}
