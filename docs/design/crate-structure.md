# Crate Structure

> Part of [Mayara Architecture](architecture.md)

This document describes the current crate structure as of December 2025.

---

## Overview

```
mayara/
├── mayara-core/                    # Platform-independent radar library
│   └── src/
│       ├── lib.rs                  # Re-exports: Brand, IoProvider, RadarLocator, controllers, etc.
│       ├── io.rs                   # IoProvider trait (UDP/TCP abstraction)
│       ├── locator.rs              # RadarLocator (multi-brand discovery)
│       ├── connection.rs           # ConnectionState, ConnectionManager, furuno login
│       ├── state.rs                # RadarState, PowerState (control values)
│       ├── brand.rs                # Brand enum (Furuno, Navico, Raymarine, Garmin)
│       ├── radar.rs                # RadarDiscovery struct
│       ├── error.rs                # ParseError type
│       ├── dual_range.rs           # Dual-range controller logic
│       │
│       ├── controllers/            # ★ UNIFIED BRAND CONTROLLERS ★
│       │   ├── mod.rs              # Re-exports all controllers
│       │   ├── furuno.rs           # FurunoController (TCP login + commands)
│       │   ├── navico.rs           # NavicoController (UDP multicast)
│       │   ├── raymarine.rs        # RaymarineController (Quantum/RD)
│       │   └── garmin.rs           # GarminController (UDP)
│       │
│       ├── protocol/               # Wire protocol (encoding/decoding)
│       │   ├── furuno/
│       │   │   ├── mod.rs          # Beacon parsing, spoke parsing, constants
│       │   │   ├── command.rs      # Format functions (format_gain_command, etc.)
│       │   │   ├── dispatch.rs     # Control dispatch (ID → wire command)
│       │   │   └── report.rs       # TCP response parsing
│       │   ├── navico.rs           # Navico: report parsing + nav packet formatting
│       │   ├── raymarine.rs        # Raymarine protocol
│       │   └── garmin.rs           # Garmin protocol
│       │
│       ├── models/                 # Radar model database
│       │   ├── furuno.rs           # DRS4D-NXT, DRS6A-NXT, etc. (ranges, controls)
│       │   ├── navico.rs           # HALO, 4G, 3G, BR24
│       │   ├── raymarine.rs        # Quantum, RD series
│       │   └── garmin.rs           # xHD series
│       │
│       ├── capabilities/           # Control definitions
│       │   ├── controls.rs         # 40+ definitions + batch getters (get_base_*, get_all_*)
│       │   └── builder.rs          # Capability manifest builder
│       │
│       ├── arpa/                   # ARPA target tracking
│       │   ├── detector.rs         # Contour detection
│       │   ├── tracker.rs          # Kalman filter tracking
│       │   ├── cpa.rs              # CPA/TCPA calculation
│       │   └── ...
│       │
│       ├── trails/                 # Target trail history
│       └── guard_zones/            # Guard zone alerting
│
├── mayara-server/                  # Standalone native server
│   └── src/
│       ├── main.rs                 # Entry point, tokio runtime
│       ├── lib.rs                  # Session, Cli, VERSION exports
│       ├── tokio_io.rs             # TokioIoProvider (implements IoProvider)
│       ├── core_locator.rs         # CoreLocatorAdapter (wraps mayara-core RadarLocator)
│       ├── locator.rs              # Legacy platform-specific locator
│       ├── web.rs                  # Axum HTTP/WebSocket handlers
│       ├── settings.rs             # SharedControls wrapper for radar state
│       ├── control_factory.rs      # Batch control builders (uses core get_base_*, get_all_*)
│       ├── storage.rs              # Local applicationData storage
│       ├── navdata.rs              # NMEA/SignalK navigation input
│       │
│       ├── brand/                  # Brand-specific async adapters
│       │   ├── furuno/             # Async report/data receivers, delegates to core
│       │   ├── navico/             # report.rs + info.rs use core protocol/navico.rs
│       │   ├── raymarine/          # Async report/data receivers, delegates to core
│       │   └── garmin/             # Discovery only (controller integration pending)
│       │
│       └── recording/              # Radar recording and playback
│           ├── mod.rs              # Module exports
│           ├── file_format.rs      # .mrr binary format read/write
│           ├── recorder.rs         # Subscribes to broadcast, writes .mrr files
│           ├── player.rs           # Reads .mrr, emits as virtual radar
│           └── manager.rs          # File listing, metadata, CRUD operations
│
├── mayara-signalk-wasm/            # SignalK WASM plugin
│   └── src/
│       ├── lib.rs                  # WASM entry point, plugin exports
│       ├── wasm_io.rs              # WasmIoProvider (implements IoProvider)
│       ├── locator.rs              # Re-exports RadarLocator from mayara-core
│       ├── radar_provider.rs       # RadarProvider (needs update to unified controllers)
│       ├── spoke_receiver.rs       # UDP spoke data receiver
│       └── signalk_ffi.rs          # SignalK FFI bindings
│
├── mayara-gui/                     # Shared web GUI assets
│   ├── index.html                  # Landing page with radar list
│   ├── viewer.html                 # Radar PPI display page
│   ├── control.html                # Radar controls panel
│   ├── recordings.html             # Recording/playback control page
│   ├── mayara.js                   # Main entry, VanJS components
│   ├── viewer.js                   # WebSocket spoke handling, rendering coordination
│   ├── control.js                  # Control UI, API interactions
│   ├── recordings.js               # Recording/playback UI logic
│   ├── render_webgpu.js            # WebGPU-based radar renderer (GPU-accelerated)
│   ├── api.js                      # REST/WebSocket API client, auto-detects mode
│   └── van-*.js                    # VanJS reactive UI library
│
├── mayara-server-signalk-plugin/   # SignalK plugin (connects to mayara-server)
│   ├── package.json                # npm manifest, SignalK webapp config
│   ├── build.js                    # Copies mayara-gui to public/
│   └── plugin/
│       └── index.js                # Main plugin: MayaraClient, RadarProvider
│
└── mayara-server-signalk-playbackrecordings-plugin/  # SignalK playback plugin (developer tool)
    ├── package.json                # npm manifest, SignalK webapp config
    ├── build.js                    # Copies mayara-gui (minus recordings.html), adds playback.html
    └── plugin/
        ├── index.js                # MrrPlayer, playback API endpoints
        ├── mrr-reader.js           # JavaScript port of file_format.rs
        └── public/
            └── playback.html       # Custom upload/playback UI
```

---

## Related Documents

- [Architecture Overview](architecture.md)
- [IoProvider Architecture](io-provider.md)
- [Unified Controllers](controllers.md)
