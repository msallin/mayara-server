# External Clients

> Part of [Mayara Architecture](architecture.md)

This document describes external client integrations including SignalK plugin and OpenCPN integration.

---

## The Shared API Architecture

The mayara-server REST API (`/v2/api/radars/*`) is the **shared interface** that enables multiple client applications to connect to the same radar infrastructure. All radar logic (protocol handling, ARPA tracking, signal processing) runs on mayara-server - clients are thin display and control layers.

```
                                    ┌─────────────────────┐
                                    │  mayara-server      │
                                    │  (localhost:6502)   │
                                    │                     │
                                    │  /v2/api/radars/*   │
                                    │  (REST + WebSocket) │
                                    └─────────┬───────────┘
                                              │
                                              │  HTTP + WebSocket
                    ┌─────────────────────────┼─────────────────────────┐
                    │                         │                         │
                    ▼                         ▼                         ▼
     ┌──────────────────────┐   ┌──────────────────────┐   ┌──────────────────────┐
     │   mayara-gui         │   │   mayara-server-     │   │   mayara_opencpn     │
     │   (Web Browser)      │   │   signalk-plugin     │   │   (Future)           │
     │                      │   │   (SignalK/Node.js)  │   │   (C++)              │
     │   - Direct access    │   │                      │   │                      │
     │   - WebGPU rendering │   │   - Exposes radars   │   │   - OpenGL rendering │
     │   - VanJS UI         │   │     via SignalK API  │   │   - Chart overlay    │
     └──────────────────────┘   └──────────────────────┘   └──────────────────────┘
                                              │
                                              ▼
                                ┌──────────────────────────────┐
                                │  SignalK Server              │
                                │  /signalk/v2/api/.../radars  │
                                │                              │
                                │  - Security (JWT)            │
                                │  - Multi-provider support    │
                                │  - Built-in binary streaming │
                                └──────────────────────────────┘
```

---

## mayara-server-signalk-plugin

The **mayara-server-signalk-plugin** is a native SignalK (JavaScript) plugin that:
1. Connects to mayara-server's REST API
2. Registers as a RadarProvider with SignalK's Radar API
3. Forwards spoke data via SignalK's `binaryStreamManager`

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            SignalK Server                                    │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                   mayara-server-signalk-plugin                          │ │
│  │                                                                         │ │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐ │ │
│  │  │  MayaraClient   │  │  RadarProvider  │  │    SpokeForwarder       │ │ │
│  │  │  (HTTP client)  │  │  (API methods)  │  │  (WS → emitData)        │ │ │
│  │  └────────┬────────┘  └────────┬────────┘  └────────────┬────────────┘ │ │
│  │           │                    │                        │              │ │
│  └───────────┼────────────────────┼────────────────────────┼──────────────┘ │
│              │   radarApi.register()      binaryStreamManager.emitData()    │
│              │                    │                        │                │
│  ┌───────────┼────────────────────┼────────────────────────┼──────────────┐ │
│  │           │        SignalK Radar API v2                 │              │ │
│  │           │   /signalk/v2/api/vessels/self/radars/*     │              │ │
│  │           │   Security: JWT via authorizeWS()           │              │ │
│  └───────────┼────────────────────────────────────────────────────────────┘ │
└──────────────┼──────────────────────────────────────────────────────────────┘
               │ HTTP + WebSocket
               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            mayara-server                                     │
│              /v2/api/radars/*            /v2/api/radars/*/spokes             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Key Features:**
- Pure JavaScript (no native dependencies beyond `ws`)
- Implements `RadarProviderMethods` interface from SignalK
- Uses SignalK's built-in `binaryStreamManager` for spoke streaming (no custom proxy)
- Auto-discovery of radars connected to mayara-server
- Auto-reconnection on network failures
- Embeds mayara-gui for web display

**Plugin Location:** `mayara-server-signalk-plugin/` (separate repository)

**Why NOT embed mayara-core in the plugin?**
- SignalK's WASM plugin already provides embedded radar support via mayara-signalk-wasm
- mayara-server-signalk-plugin is for deployments where mayara-server runs separately
- Separation allows mayara-server to run on different hardware (e.g., dedicated radar PC)
- Single mayara-server can serve multiple clients (SignalK, direct browser, future OpenCPN)

---

## OpenCPN Plugin Integration

The `mayara-server-opencpn-plugin` is a C++ plugin for OpenCPN that connects to mayara-server via the same REST/WebSocket API used by the SignalK plugin. This provides radar display capabilities within OpenCPN without requiring the radar_pi plugin's direct protocol implementations.

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         OpenCPN                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              mayara-server-opencpn-plugin               │    │
│  │  ┌──────────────┐  ┌─────────────┐  ┌───────────────┐  │    │
│  │  │ MayaraClient │  │SpokeReceiver│  │ RadarRenderer │  │    │
│  │  │   (REST)     │  │    (WS)     │  │   (OpenGL)    │  │    │
│  │  └──────┬───────┘  └──────┬──────┘  └───────────────┘  │    │
│  └─────────┼─────────────────┼────────────────────────────┘    │
│            │                 │                                   │
└────────────┼─────────────────┼───────────────────────────────────┘
             │ HTTP            │ WebSocket
             │                 │ (protobuf)
┌────────────▼─────────────────▼───────────────────────────────────┐
│                      mayara-server                                │
│                     localhost:6502                                │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                      RadarEngine                             │ │
│  │  ┌─────────┐ ┌─────────┐ ┌──────────┐ ┌─────────┐          │ │
│  │  │ Furuno  │ │ Navico  │ │Raymarine │ │ Garmin  │          │ │
│  │  └────┬────┘ └────┬────┘ └────┬─────┘ └────┬────┘          │ │
│  └───────┼───────────┼───────────┼────────────┼────────────────┘ │
└──────────┼───────────┼───────────┼────────────┼──────────────────┘
           │           │           │            │
      ┌────▼───┐  ┌────▼───┐  ┌────▼────┐  ┌────▼───┐
      │ DRS4D  │  │ HALO   │  │ Quantum │  │  xHD   │
      └────────┘  └────────┘  └─────────┘  └────────┘
```

### API Usage

The plugin uses the same endpoints as the SignalK plugin:

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/v2/api/radars` | Discover radars |
| GET | `/v2/api/radars/{id}/capabilities` | Get radar specs |
| GET | `/v2/api/radars/{id}/state` | Get current settings |
| PUT | `/v2/api/radars/{id}/controls/{ctrl}` | Set control value |
| WS | `/v2/api/radars/{id}/spokes` | Binary spoke stream |
| GET | `/v2/api/radars/{id}/targets` | Get ARPA targets |

### Display Modes

1. **Chart Overlay**: Renders radar on OpenCPN's chart canvas using
   `RenderGLOverlayMultiCanvas()` callback with OpenGL shaders

2. **PPI Window**: Separate `wxGLCanvas` window with traditional
   radar PPI display, range rings, and heading marker

### Benefits over radar_pi

| Aspect | radar_pi | mayara-server plugin |
|--------|----------|---------------------|
| Protocol handling | In plugin | In server |
| Multi-client | No | Yes (multiple UIs) |
| Platform code | Per radar brand | Single API client |
| Updates | Plugin rebuild | Server update only |
| Remote radar | No | Yes (server can run elsewhere) |

### Source Repository

- Plugin: https://github.com/MarineYachtRadar/mayara-server-opencpn-plugin
- Documentation: Included in plugin as AsciiDoc manual

---

## Client Comparison

| Client | Language | Use Case | Radar Logic |
|--------|----------|----------|-------------|
| **mayara-gui** | JavaScript | Direct browser access | mayara-server |
| **mayara-signalk-wasm** | Rust/WASM | Embedded in SignalK | mayara-core (in WASM) |
| **mayara-server-signalk-plugin** | JavaScript | SignalK + remote mayara-server | mayara-server |
| **mayara_opencpn** (future) | C++ | OpenCPN chart plotter | mayara-server |

---

## Related Documents

- [Architecture Overview](architecture.md)
- [Deployment Modes](deployment-modes.md)
- [Recording and Playback](recording-playback.md)
