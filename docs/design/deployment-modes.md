# Deployment Modes

> Part of [Mayara Architecture](architecture.md)

This document describes the two deployment modes: SignalK WASM Plugin and Standalone Server.

---

## Mode 1: SignalK WASM Plugin

> **Note:** The WASM plugin is now fully integrated with the unified RadarEngine
> architecture from mayara-core. It shares the same controllers, ARPA, guard zones,
> trails, and dual-range logic as the server.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    SignalK Server (Node.js)                                  │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │              WASM Runtime (wasmer)                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │ │
│  │  │         mayara-signalk-wasm                                       │  │ │
│  │  │                                                                   │  │ │
│  │  │  ┌──────────────────┐  ┌───────────────────────────────────────┐ │  │ │
│  │  │  │  WasmIoProvider  │  │   RadarLocator (from mayara-core)     │ │  │ │
│  │  │  │  (FFI sockets)   │──│   SAME CODE AS SERVER                 │ │  │ │
│  │  │  └──────────────────┘  └───────────────────────────────────────┘ │  │ │
│  │  │                                                                   │  │ │
│  │  │  ┌──────────────────────────────────────────────────────────┐    │  │ │
│  │  │  │         Unified Controllers (from mayara-core)            │    │  │ │
│  │  │  │  FurunoController   │ NavicoController   (SAME CODE!)     │    │  │ │
│  │  │  │  RaymarineController│ GarminController   (AS SERVER!)     │    │  │ │
│  │  │  └──────────────────────────────────────────────────────────┘    │  │ │
│  │  └──────────────────────────────────────────────────────────────────┘  │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Characteristics:**
- Runs inside SignalK's WASM sandbox
- Uses SignalK FFI for all network I/O via WasmIoProvider
- Poll-based (no async runtime in WASM)
- **Same RadarLocator AND Controllers as server** (all 4 brands!)
- Uses RadarEngine from mayara-core for unified feature management

### Spoke Reduction

The WASM plugin reduces Furuno's native 8192 spokes to 512 per revolution. This is necessary because SignalK's WebSocket cannot sustain the data rate of full-resolution spokes (code 1008 "Client cannot keep up"). The `spokes_per_revolution` in capabilities is adjusted to match the actual output, ensuring the GUI correctly maps spoke angles to 360 degrees.

---

## Mode 2: Standalone Server

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    mayara-server (Rust)                                      │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     CoreLocatorAdapter                               │    │
│  │  ┌──────────────────┐  ┌───────────────────────────────────────┐    │    │
│  │  │  TokioIoProvider │  │   RadarLocator (from mayara-core)     │    │    │
│  │  │  (tokio sockets) │──│   SAME CODE AS WASM                   │    │    │
│  │  └──────────────────┘  └───────────────────────────────────────┘    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │   Brand Adapters (brand/) + Core Controllers (controllers/)          │    │
│  │   - Async receivers in brand/ handle tokio sockets, spoke streaming  │    │
│  │   - Delegate control commands to mayara-core unified controllers     │    │
│  │   - TokioIoProvider implements IoProvider for controller I/O         │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │              Axum Router (web.rs)                                    │    │
│  │   /radars/*, /targets/*, static files (rust-embed from mayara-gui/) │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Characteristics:**
- Native Rust binary with tokio async runtime
- Direct network I/O via TokioIoProvider
- Axum web server hosts API + GUI
- **Same RadarLocator AND Controllers as WASM** (from mayara-core)
- **Same API paths as SignalK** → same GUI works unchanged

---

## Spoke Resolution Comparison

The server and WASM handle different spoke resolutions due to transport constraints:

| Platform | Spokes/Revolution | Reason |
|----------|------------------|--------|
| **mayara-server** | 8192 (native) | Direct WebSocket to browser can sustain high data rate |
| **mayara-signalk-wasm** | 512 (reduced) | SignalK WebSocket has rate limiting (code 1008) |

**WASM Spoke Reduction Logic** (`spoke_receiver.rs`):
1. Furuno sends 8192 spokes per revolution
2. WASM accumulates 16 consecutive spokes
3. Combines using `max()` per pixel (preserves radar targets)
4. Emits 1 combined spoke with angle `original_angle / 16`
5. Results in 512 spokes/revolution (8192 / 16)

**Critical:** The `spokes_per_revolution` in capabilities must match the actual output.
The GUI uses this value to map spoke angles to 360 degrees:
- Server: `spokes_per_revolution: 8192`, angles 0-8191
- WASM: `spokes_per_revolution: 512`, angles 0-511

The WASM uses `build_capabilities_from_model_with_spokes()` to override the model's native spoke count with the reduced output count.

---

## Related Documents

- [Architecture Overview](architecture.md)
- [IoProvider Architecture](io-provider.md)
- [External Clients](external-clients.md)
