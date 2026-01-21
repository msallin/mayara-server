# Mayara Architecture

> This document describes the architecture of the Mayara radar system,
> showing what is shared between deployment modes and the path to maximum code reuse.

---

## Document Structure

This architecture documentation is split into the following topics:

| Document | Description |
|----------|-------------|
| [Crate Structure](crate-structure.md) | Current crate layout and module organization |
| [IoProvider Architecture](io-provider.md) | Platform-independent I/O abstraction for WASM and native |
| [Unified Controllers](controllers.md) | Brand-specific controllers shared between platforms |
| [RadarEngine](radar-engine.md) | Unified feature management (ARPA, trails, guard zones) |
| [Deployment Modes](deployment-modes.md) | SignalK WASM plugin vs standalone server |
| [External Clients](external-clients.md) | SignalK plugin, OpenCPN integration |
| [Recording and Playback](recording-playback.md) | .mrr file format and playback system |
| [Debug Infrastructure](debug-infrastructure.md) | Dev-mode protocol analysis tools |
| [Data Flows](data-flows.md) | Control, spoke, and discovery data flows |
| [Batch Control Initialization](batch-control-init.md) | Dynamic control discovery and persistence |
| [Implementation Status](implementation-status.md) | Current status and architecture evolution |
| [Known Issues](known-issues.md) | Known issues and workarounds |

---

## FUNDAMENTAL PRINCIPLE: mayara-core is the Single Source of Truth

**This is the most important architectural concept in Mayara.**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        mayara-core (THE DATABASE)                            │
│                                                                              │
│   Contains ALL knowledge about radars:                                       │
│   - Model database (ranges, spokes, capabilities per model)                  │
│   - Control definitions (what controls exist, their types, min/max, units)   │
│   - Protocol specifications (wire format, parsing, command dispatch)         │
│   - Feature flags (doppler, dual-range, no-transmit zones, etc.)            │
│   - Connection state machine (platform-independent)                          │
│   - I/O abstraction (IoProvider trait)                                      │
│   - RadarLocator (discovery logic)                                          │
│                                                                              │
│   THIS IS THE ONLY PLACE WHERE RADAR LOGIC IS DEFINED.                      │
│   SERVER AND WASM ARE THIN I/O ADAPTERS AROUND CORE.                        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ adapters implement IoProvider
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           I/O Provider Layer                                 │
│                                                                              │
│  ┌─────────────────────────┐          ┌─────────────────────────┐           │
│  │    TokioIoProvider      │          │     WasmIoProvider      │           │
│  │    (mayara-server)      │          │  (mayara-signalk-wasm)  │           │
│  │                         │          │                         │           │
│  │  Wraps tokio sockets    │          │  Wraps SignalK FFI      │           │
│  │  in poll-based API      │          │  socket calls           │           │
│  └─────────────────────────┘          └─────────────────────────┘           │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ exposes via
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           REST API (SignalK-compatible)                      │
│                                                                              │
│   GET /radars/{id}/capabilities    ← Returns model info from mayara-core    │
│   GET /radars/{id}/state           ← Current control values                 │
│   PUT /radars/{id}/controls/{id}   ← Set control values                     │
│                                                                              │
│   The API is the CONTRACT. All clients use ONLY the API.                    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ consumed by
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ALL CLIENTS                                     │
│                                                                              │
│   - WebGUI (mayara-gui/)           - Reads /capabilities to know what       │
│   - mayara-server internal logic     controls to display                    │
│   - Future: mayara_opencpn         - Dynamically builds UI from API         │
│   - Future: mobile apps            - NEVER hardcodes radar capabilities     │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## What This Means in Practice

1. **mayara-core defines everything:**
   - All radar models and their specifications
   - All control types (gain, sea, rain, dopplerMode, etc.)
   - Valid ranges per model
   - Available features per model
   - Wire protocol encoding/decoding
   - **Command dispatch** (control ID → wire command)
   - **Connection state machine** (Disconnected → Connecting → Connected → Active)

2. **mayara-server and mayara-signalk-wasm are thin adapters:**
   - Implement `IoProvider` trait for their platform
   - Run the **same** RadarLocator code from mayara-core
   - Use the **same** dispatch functions for control commands
   - No hardcoded control names, range tables, or protocol details

3. **The REST API is the contract:**
   - `/capabilities` returns what the radar can do (from mayara-core)
   - Clients build their UI dynamically from this response
   - Same WebGUI works for ANY radar brand because it follows the API

4. **Adding a new control:**
   - Add definition to `mayara-core/capabilities/controls.rs`
   - Add dispatch entry in `mayara-core/protocol/{brand}/dispatch.rs`
   - Add to model's control list in `mayara-core/models/{brand}.rs`
   - **Server and WASM automatically pick it up - no changes needed!**

---

## Architecture Diagram: Full Picture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              mayara-core                                     │
│                    (Pure Rust, no I/O, WASM-compatible)                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────┐ ┌───────────────┐ ┌───────────────┐ ┌───────────────┐   │
│  │  protocol/    │ │   models/     │ │ capabilities/ │ │   state.rs    │   │
│  │  - furuno/    │ │ - furuno.rs   │ │ - controls.rs │ │   RadarState  │   │
│  │    - dispatch │ │ - navico.rs   │ │   get_base_*  │ │   PowerState  │   │
│  │    - command  │ │ - raymarine   │ │   get_all_*   │ │               │   │
│  │    - report   │ │ - garmin.rs   │ │ - builder.rs  │ │               │   │
│  │  - navico.rs  │ │               │ │               │ │               │   │
│  │    (parse +   │ │               │ │               │ │               │   │
│  │     format)   │ │               │ │               │ │               │   │
│  │  - raymarine  │ │               │ │               │ │               │   │
│  │  - garmin.rs  │ │               │ │               │ │               │   │
│  └───────────────┘ └───────────────┘ └───────────────┘ └───────────────┘   │
│                                                                              │
│  ┌───────────────┐ ┌───────────────┐ ┌───────────────┐ ┌───────────────┐   │
│  │  io.rs        │ │ locator.rs    │ │ connection.rs │ │  arpa/        │   │
│  │  IoProvider   │ │ RadarLocator  │ │ ConnManager   │ │  trails/      │   │
│  │  trait        │ │ (discovery)   │ │ ConnState     │ │  guard_zones/ │   │
│  └───────────────┘ └───────────────┘ └───────────────┘ └───────────────┘   │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    controllers/  (★ UNIFIED ★)                       │   │
│  │   FurunoController │ NavicoController │ RaymarineController │ Garmin │   │
│  │   (TCP login)      │ (UDP multicast)  │ (Quantum/RD)        │ (UDP)  │   │
│  │                                                                      │   │
│  │   ALL controllers use IoProvider - SAME code on server AND WASM!    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
     ┌────────────────────────────┐    ┌────────────────────────────┐
     │   mayara-signalk-wasm      │    │       mayara-server        │
     │      (WASM + FFI)          │    │    (Native + tokio)        │
     ├────────────────────────────┤    ├────────────────────────────┤
     │                            │    │                            │
     │  wasm_io.rs:               │    │  tokio_io.rs:              │
     │  - WasmIoProvider          │    │  - TokioIoProvider         │
     │  - impl IoProvider         │    │  - impl IoProvider         │
     │                            │    │                            │
     │  locator.rs:               │    │  core_locator.rs:          │
     │  - Re-exports RadarLocator │    │  - CoreLocatorAdapter      │
     │    from mayara-core        │    │  - Wraps RadarLocator      │
     │                            │    │                            │
     │  radar_provider.rs:        │    │  brand/:                   │
     │  - Uses controllers from   │    │  - Can use core controllers│
     │    mayara-core directly!   │    │    with TokioIoProvider    │
     │  - FurunoController        │    │  - OR async wrappers       │
     │  - NavicoController        │    │                            │
     │  - RaymarineController     │    │  web.rs:                   │
     │  - GarminController        │    │  - Axum handlers           │
     │                            │    │                            │
     │  signalk_ffi.rs:           │    │  storage.rs:               │
     │  - FFI bindings            │    │  - Local applicationData   │
     └────────────────────────────┘    └────────────────────────────┘
                    │                               │
                    ▼                               ▼
     ┌────────────────────────────┐    ┌────────────────────────────┐
     │     SignalK Server         │    │     Axum HTTP Server       │
     │                            │    │                            │
     │  Routes /radars/* to       │    │  /radars/*  (same API!)    │
     │  WASM RadarProvider        │    │  Static files (same GUI!)  │
     └────────────────────────────┘    └────────────────────────────┘
                    │                               │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                     ┌────────────────────────────┐
                     │         mayara-gui/        │
                     │     (shared web assets)    │
                     │                            │
                     │  Works in ANY mode!        │
                     │  api.js auto-detects       │
                     └────────────────────────────┘
```

---

## Benefits of This Architecture

| Benefit | Description |
|---------|-------------|
| **Single source of truth** | All radar logic in mayara-core |
| **Fixes apply everywhere** | Bug fixed in core → fixed in WASM and Server |
| **No code duplication** | Same RadarLocator, same controllers, same dispatch |
| **All 4 brands everywhere** | Furuno, Navico, Raymarine, Garmin work on WASM AND Server |
| **Easy to add features** | Add to core, both platforms get it automatically |
| **Testable** | Core is pure Rust, mock IoProvider for unit tests |
| **WASM-compatible** | Core has zero tokio dependencies |
| **Same GUI** | Works unchanged with SignalK or Standalone |
| **Same API** | Clients don't know which backend they're talking to |

---

## Testing Strategy

The unified architecture enables effective testing at multiple levels:

### Unit Tests (mayara-core)

Core logic can be tested without real hardware using mock IoProvider:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    struct MockIoProvider {
        sent_data: Vec<(String, u16, Vec<u8>)>,
    }

    impl IoProvider for MockIoProvider {
        fn udp_send_to(&mut self, _socket: &UdpSocketHandle, data: &[u8],
                       addr: &str, port: u16) -> Result<usize, IoError> {
            self.sent_data.push((addr.to_string(), port, data.to_vec()));
            Ok(data.len())
        }
        // ... other methods
    }

    #[test]
    fn test_gain_command_quantum() {
        let mut io = MockIoProvider { sent_data: vec![] };
        let mut controller = RaymarineController::new(
            "test", "192.168.1.100", 50100, "239.0.0.1", 50100,
            RaymarineVariant::Quantum, false
        );

        controller.set_gain(&mut io, 50, false);

        assert_eq!(io.sent_data.len(), 1);
        let (addr, port, data) = &io.sent_data[0];
        assert_eq!(addr, "192.168.1.100");
        // Verify Quantum command format
        assert_eq!(data[2], 0x28);  // Quantum magic byte
    }
}
```

### Integration Tests (mayara-server)

Test REST API endpoints with mock radar:

```rust
#[tokio::test]
async fn test_radar_capabilities_endpoint() {
    // Start server with test radar registered
    let app = create_test_app();

    let response = app
        .oneshot(Request::get("/v2/api/radars/test-radar/capabilities").body(Body::empty())?)
        .await?;

    assert_eq!(response.status(), StatusCode::OK);
    let body: serde_json::Value = serde_json::from_slice(&body_bytes(response).await)?;
    assert!(body["controls"].is_array());
}
```

### Replay Testing

Recorded radar data can be replayed to test parsing and processing:

```bash
# Record live radar traffic
tcpdump -i eth0 -w capture.pcap port 50100 or port 50102

# Replay in test mode
mayara-server --replay capture.pcap
```

The `receiver.replay` flag prevents controller creation during replay, allowing spoke processing to be tested independently.

---

## Related Documents

- [Forked Dependencies](forked-dependencies.md) - Why we use forked versions of nmea-parser and tungstenite
