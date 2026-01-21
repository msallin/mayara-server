# Unified Controllers Architecture

> Part of [Mayara Architecture](architecture.md)

This document describes the unified controller system in `mayara-core/controllers/` that eliminates code duplication between server and WASM.

---

## Overview

The most significant architectural advancement is the **unified controller system**. This ensures identical behavior across platforms.

---

## Controller Design Principles

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Controller Design Pattern                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  1. Poll-based (not async) → works in WASM without runtime                  │
│  2. IoProvider abstraction → no direct socket calls                         │
│  3. State machine → handles connect/disconnect/reconnect                    │
│  4. Brand-specific protocol → TCP (Furuno) or UDP (Navico/Raymarine/Garmin) │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                      Controller Interface                               │ │
│  │                                                                         │ │
│  │  fn new(radar_id, address, ...) -> Self                                │ │
│  │  fn poll<I: IoProvider>(&mut self, io: &mut I) -> bool                 │ │
│  │  fn is_connected(&self) -> bool                                        │ │
│  │  fn state(&self) -> ControllerState                                    │ │
│  │                                                                         │ │
│  │  // Control setters (all take IoProvider)                              │ │
│  │  fn set_power<I: IoProvider>(&mut self, io: &mut I, transmit: bool)    │ │
│  │  fn set_range<I: IoProvider>(&mut self, io: &mut I, meters: u32)       │ │
│  │  fn set_gain<I: IoProvider>(&mut self, io: &mut I, value: u32, auto)   │ │
│  │  fn set_sea<I: IoProvider>(&mut self, io: &mut I, value: u32, auto)    │ │
│  │  fn set_rain<I: IoProvider>(&mut self, io: &mut I, value: u32, auto)   │ │
│  │  ...                                                                    │ │
│  │                                                                         │ │
│  │  fn shutdown<I: IoProvider>(&mut self, io: &mut I)                     │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Controller State Machines

Each controller manages its own connection state:

```
                    ┌──────────────┐
                    │ Disconnected │ ◄──────────────────────────────┐
                    └──────┬───────┘                                │
                           │ poll() creates sockets                 │
                           ▼                                        │
                    ┌──────────────┐                                │
                    │  Listening   │  (UDP: waiting for reports)    │
                    │  Connecting  │  (TCP: waiting for connect)    │
                    └──────┬───────┘                                │
                           │ reports received / TCP connected       │
                           ▼                                        │
                    ┌──────────────┐                                │
                    │  Connected   │  (ready for commands)          │
                    └──────┬───────┘                                │
                           │ connection lost / timeout              │
                           └────────────────────────────────────────┘
```

---

## Brand-Specific Details

| Brand | Protocol | Connection | Special Features |
|-------|----------|------------|------------------|
| **Furuno** | TCP | Login sequence (root) | NXT Doppler modes, ~30 controls |
| **Navico** | UDP multicast | Report multicast join | BR24/3G/4G/HALO, Doppler (HALO) |
| **Raymarine** | UDP | Report multicast | Quantum (solid-state) vs RD (magnetron) |
| **Garmin** | UDP multicast | Report multicast | xHD series, simple protocol |

---

## RaymarineController Variants

Raymarine radars come in two fundamentally different types with incompatible command formats:

```
┌────────────────────────────────────────────────────────────────────────────┐
│                        RaymarineController                                  │
│  (mayara-core/controllers/raymarine.rs)                                    │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  RaymarineVariant::Quantum (Solid-State)                                   │
│  ├── Command format: [opcode_lo, opcode_hi, 0x28, 0x00, 0x00, value, ...]  │
│  ├── One-byte values: quantum_one_byte_command(opcode, value)              │
│  ├── Two-byte values: quantum_two_byte_command(opcode, value)              │
│  └── Models: Quantum, Quantum 2                                            │
│                                                                             │
│  RaymarineVariant::RD (Magnetron)                                          │
│  ├── Command format: [0x00, 0xc1, lead_bytes..., value, 0x00, ...]        │
│  ├── Standard: rd_standard_command(lead, value)                            │
│  ├── On/Off: rd_on_off_command(lead, on_off)                              │
│  └── Models: RD418D, RD418HD, RD424D, RD424HD, RD848                       │
│                                                                             │
│  The server creates the correct variant when model is detected:            │
│    RaymarineController::new(..., RaymarineVariant::Quantum, ...)           │
│    RaymarineController::new(..., RaymarineVariant::RD, ...)                │
│                                                                             │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Usage Example (WASM)

```rust
// mayara-signalk-wasm/src/radar_provider.rs

use mayara_core::controllers::{
    FurunoController, NavicoController, RaymarineController, GarminController,
};
use mayara_core::Brand;

struct RadarProvider {
    io: WasmIoProvider,
    furuno_controllers: BTreeMap<String, FurunoController>,
    navico_controllers: BTreeMap<String, NavicoController>,
    raymarine_controllers: BTreeMap<String, RaymarineController>,
    garmin_controllers: BTreeMap<String, GarminController>,
}

impl RadarProvider {
    fn poll(&mut self) {
        // Poll all controllers - same code regardless of platform!
        for controller in self.furuno_controllers.values_mut() {
            controller.poll(&mut self.io);
        }
        for controller in self.navico_controllers.values_mut() {
            controller.poll(&mut self.io);
        }
        // ... etc
    }

    fn set_gain(&mut self, radar_id: &str, value: u32, auto: bool) {
        if let Some(c) = self.furuno_controllers.get_mut(radar_id) {
            c.set_gain(&mut self.io, value, auto);
        } else if let Some(c) = self.navico_controllers.get_mut(radar_id) {
            c.set_gain(&mut self.io, value, auto);
        }
        // ... etc
    }
}
```

---

## Server Integration Pattern

The server's `brand/` modules wrap core controllers with async/tokio integration:

```rust
// mayara-server/src/brand/raymarine/report.rs (simplified)

use mayara_core::controllers::{RaymarineController, RaymarineVariant};
use crate::tokio_io::TokioIoProvider;

pub struct RaymarineReportReceiver {
    controller: Option<RaymarineController>,  // Core controller
    io: TokioIoProvider,                       // Platform I/O adapter
    // ... other fields for spoke data, trails, etc.
}

impl RaymarineReportReceiver {
    // When model is detected, create the appropriate variant
    fn on_model_detected(&mut self, model: &RaymarineModel) {
        self.controller = Some(RaymarineController::new(
            &self.key,
            &self.info.send_command_addr.ip().to_string(),
            self.info.send_command_addr.port(),
            &self.info.report_addr.ip().to_string(),
            self.info.report_addr.port(),
            if model.is_quantum() { RaymarineVariant::Quantum }
            else { RaymarineVariant::RD },
            model.doppler,
        ));
    }

    // Control requests come through ControlValue channel
    async fn send_control_to_radar(&mut self, cv: &ControlValue) -> Result<(), RadarError> {
        let controller = self.controller.as_mut()
            .ok_or_else(|| RadarError::CannotSetControlType("Controller not initialized".into()))?;

        match cv.id.as_str() {
            "power" => controller.set_power(&mut self.io, cv.value as u8),
            "range" => controller.set_range(&mut self.io, cv.value as u32),
            "gain" => controller.set_gain(&mut self.io, cv.value as u32, cv.auto.unwrap_or(false)),
            // ... 20+ more controls
            _ => return Err(RadarError::CannotSetControlType(cv.id.clone())),
        }
        Ok(())
    }
}
```

**Key insight:** The server's brand modules are now thin dispatchers that:
1. Create core controllers when radar model is detected
2. Route control requests to the appropriate core controller method
3. Handle async spoke data reception (still server-specific)
4. Manage WebSocket broadcasting to clients

---

## Benefits of Unified Controllers

| Benefit | Description |
|---------|-------------|
| **Single source of truth** | Fix bugs once, fixed everywhere |
| **Consistent behavior** | WASM and server behave identically |
| **Easier testing** | Mock IoProvider for unit tests |
| **Reduced code size** | ~1500 lines shared vs ~3000 lines duplicated |
| **Faster feature development** | Add control to core, works on all platforms |

---

## Related Documents

- [Architecture Overview](architecture.md)
- [IoProvider Architecture](io-provider.md)
- [RadarEngine](radar-engine.md)
