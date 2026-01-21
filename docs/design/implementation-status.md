# Implementation Status

> Part of [Mayara Architecture](architecture.md)

This document tracks the implementation status of components as of December 2025.

---

## Fully Implemented (Server + WASM)

| Component | Location | Notes |
|-----------|----------|-------|
| **Protocol parsing** | mayara-core/protocol/ | All 4 brands: Furuno, Navico, Raymarine, Garmin |
| **Protocol formatting** | mayara-core/protocol/navico.rs | Navigation packets (heading/SOG/COG) |
| **Model database** | mayara-core/models/ | All models with ranges, spokes, capabilities |
| **Control definitions** | mayara-core/capabilities/ | 40+ controls (v5 API) |
| **Batch control init** | mayara-core/capabilities/controls.rs | get_base_controls_for_brand(), get_all_controls_for_model() |
| **IoProvider trait** | mayara-core/io.rs | Platform-independent I/O abstraction |
| **RadarLocator** | mayara-core/locator.rs | Multi-brand discovery via IoProvider |
| **ConnectionManager** | mayara-core/connection.rs | State machine, backoff, Furuno login |
| **RadarState types** | mayara-core/state.rs | Control values, update_from_response() |
| **Dispatch functions** | mayara-core/protocol/furuno/dispatch.rs | Control ID → wire command routing |
| **Unified Controllers** | mayara-core/controllers/ | All 4 brands: FurunoController, NavicoController, RaymarineController, GarminController |
| **RadarEngine** | mayara-core/engine/ | Unified management of controllers + feature processors |
| **ARPA tracking** | mayara-core/arpa/ | Kalman filter, CPA/TCPA, contour detection |
| **Trails history** | mayara-core/trails/ | Target position storage |
| **Guard zones** | mayara-core/guard_zones/ | Zone alerting logic |
| **Dual-range** | mayara-core/dual_range.rs | Dual-range controller for supported models |
| **TokioIoProvider** | mayara-server/tokio_io.rs | Tokio sockets implementing IoProvider |
| **CoreLocatorAdapter** | mayara-server/core_locator.rs | Async wrapper for RadarLocator |
| **Standalone server** | mayara-server/ | Full functionality, uses RadarEngine |
| **Web GUI** | mayara-gui/ | WebGPU rendering, VanJS framework |
| **Local storage API** | mayara-server/storage.rs | SignalK-compatible applicationData |
| **WasmIoProvider** | mayara-signalk-wasm/wasm_io.rs | SignalK FFI socket wrapper |
| **SignalK WASM plugin** | mayara-signalk-wasm/ | Uses RadarEngine, thin shell around core |

---

## Server Brand Controller Integration

The server's brand modules now delegate to unified core controllers:

| Brand | Core Controller | Server Integration | Status |
|-------|-----------------|-------------------|--------|
| **Furuno** | `FurunoController` (TCP login + commands) | `brand/furuno/report.rs` uses core | Complete |
| **Navico** | `NavicoController` (UDP multicast) | `report.rs` + `info.rs` use core protocol | Complete |
| **Raymarine** | `RaymarineController` (Quantum/RD) | `brand/raymarine/report.rs` uses core | Complete |
| **Garmin** | `GarminController` (UDP) | Core ready, server uses legacy locator | Partial |

The server's `brand/` modules still handle:
- Async spoke data reception (tokio streams)
- Radar discovery and lifecycle management
- Control value caching and broadcasting
- WebSocket spoke streaming to clients
- Navigation data sending (Navico `info.rs` uses core formatting functions)

---

## Recently Implemented

| Component | Notes |
|-----------|-------|
| mayara-server-signalk-plugin | Native JS plugin connecting SignalK to mayara-server (see [External Clients](external-clients.md)) |
| Recording/Playback (mayara-server) | .mrr file format, recording, playback, REST API (see [Recording and Playback](recording-playback.md)) |
| recordings.html/js (mayara-gui) | Web UI for recording and playback control |
| mayara-server-signalk-playbackrecordings-plugin | SignalK playback plugin for developers (no mayara-server required) |

---

## Not Yet Implemented

| Component | Notes |
|-----------|-------|
| mayara_opencpn plugin | OpenCPN integration (see [External Clients](external-clients.md)) |
| Garmin server controller | Server still uses old locator-based approach |
| Playback speed control | Currently plays at recorded speed only |
| Playback seek | Timeline seeking not yet implemented |

---

## Architecture Evolution

The architecture evolved through several phases to achieve maximum code reuse:

### Phase 1: Server-Only (Historical)
- Each brand had its own locator, command, report, and data modules
- No sharing between brands or platforms
- Code duplication between brands (~2000+ lines per brand)

### Phase 2: Protocol Extraction
- Wire protocol parsing moved to mayara-core
- Model database (ranges, capabilities) centralized
- Control definitions unified across brands
- Server still had brand-specific controllers

### Phase 3: IoProvider Abstraction
- `IoProvider` trait created for platform-independent I/O
- `RadarLocator` moved to core (discovery logic shared)
- `TokioIoProvider` for server, `WasmIoProvider` for WASM
- Both platforms use identical discovery code

### Phase 4: Unified Controllers
- Brand controllers moved to mayara-core:
  - `FurunoController` - TCP login + command protocol
  - `NavicoController` - UDP multicast commands
  - `RaymarineController` - Quantum/RD variant handling
  - `GarminController` - UDP commands
- Server's brand modules become thin dispatchers
- WASM and server share identical control logic

### Phase 5: RadarEngine + WASM Migration (Current - December 2025)
- `RadarEngine` created in mayara-core to unify feature processors
- Server migrated from separate state types to single `SharedEngine`
- WASM plugin overhauled: discarded buggy logic, now uses RadarEngine
- Spoke reduction implemented for WASM (512 spokes vs server's 8192)
- Capabilities API updated to report actual spoke output count

### Remaining Work
- Garmin server integration (core controller exists, server still uses legacy)
- SignalK provider mode (standalone → SignalK registration)
- OpenCPN plugin (HTTP/WebSocket client)

---

## Related Documents

- [Architecture Overview](architecture.md)
- [Crate Structure](crate-structure.md)
