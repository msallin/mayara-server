# Data Flow Diagrams

> Part of [Mayara Architecture](architecture.md)

This document describes the data flows for controls, spokes, and discovery.

---

## Control Command Flow

When a user changes a control (e.g., sets gain to 50):

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Control Flow                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  WebGUI                                                                      │
│    │ PUT /radars/{id}/controls/gain {value: 50, auto: false}                │
│    ▼                                                                         │
│  Axum Handler (web.rs)                                                       │
│    │ Sends ControlValue to brand module via channel                         │
│    ▼                                                                         │
│  Brand Report Receiver (e.g., raymarine/report.rs)                          │
│    │ Receives ControlValue from channel                                     │
│    │ Calls send_control_to_radar(&cv)                                       │
│    ▼                                                                         │
│  Core Controller (controllers/raymarine.rs)                                  │
│    │ controller.set_gain(&mut io, 50, false)                                │
│    │ Builds command bytes for Quantum or RD variant                         │
│    ▼                                                                         │
│  TokioIoProvider                                                             │
│    │ io.udp_send_to(&socket, command_bytes, &radar_addr, port)              │
│    ▼                                                                         │
│  UDP Socket → Radar Hardware                                                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Spoke Data Flow

When radar sends spoke data:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Spoke Data Flow                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Radar Hardware                                                              │
│    │ UDP multicast spoke packets                                            │
│    ▼                                                                         │
│  Brand Data Receiver (e.g., raymarine/data.rs)                              │
│    │ Async tokio::net::UdpSocket.recv()                                     │
│    │ Parses frame header, decompresses spoke data                           │
│    │ Uses mayara-core protocol parsing                                       │
│    ▼                                                                         │
│  Spoke Processing                                                            │
│    │ Apply trails (mayara-core/trails/)                                     │
│    │ Convert to protobuf spoke format                                        │
│    ▼                                                                         │
│  RadarInfo.broadcast_radar_message()                                         │
│    │ Sends to all connected WebSocket clients                               │
│    ▼                                                                         │
│  WebSocket Stream                                                            │
│    │ Binary protobuf message                                                │
│    ▼                                                                         │
│  WebGUI (viewer.js)                                                          │
│    │ Decodes protobuf, renders on canvas                                    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Discovery Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Discovery Flow                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  RadarLocator (mayara-core/locator.rs)                                       │
│    │ Poll-based, runs in CoreLocatorAdapter (server) or directly (WASM)     │
│    ▼                                                                         │
│  Brand-specific beacon detection                                             │
│    │ Furuno: broadcast request → unicast response                           │
│    │ Navico: multicast join → beacon packets                                │
│    │ Raymarine: multicast join → info packets                               │
│    │ Garmin: multicast join → beacon packets                                │
│    ▼                                                                         │
│  RadarDiscovery struct created                                               │
│    │ Contains: brand, model, address, capabilities                          │
│    ▼                                                                         │
│  Server: Spawns brand-specific receiver task                                 │
│    │ Creates FurunoReportReceiver / NavicoReportReceiver / etc.             │
│    │ Receiver creates Core Controller when model confirmed                  │
│    ▼                                                                         │
│  Radar registered in Radars collection                                       │
│    │ Available via REST API /radars                                         │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Navigation Data Formatting

Navico radars require navigation data (heading, SOG, COG) to be sent as UDP multicast packets for proper HALO/4G operation. The packet formatting functions in `mayara-core/protocol/navico.rs` are pure functions that create byte arrays, enabling both server and WASM to send identical packets.

### Packet Types

| Packet | Function | Multicast Address | Purpose |
|--------|----------|-------------------|---------|
| **Heading** | `format_heading_packet()` | 236.6.7.8:50200 | Ship heading for display orientation |
| **Navigation** | `format_navigation_packet()` | 236.6.7.8:50200 | SOG + COG for trail orientation |
| **Speed** | `format_speed_packet()` | 236.6.7.5:50201 + 236.6.7.6:50201 | Speed/course for target motion |

### Packet Parsing

The same `navico.rs` file also provides packet parsing via `transmute()` methods on the packed structs:

```rust
// mayara-core/src/protocol/navico.rs

// Parsing received packets (in server's report.rs):
impl HaloHeadingPacket {
    pub fn transmute(bytes: &[u8]) -> Result<Self, &'static str>
    pub fn heading_degrees(&self) -> f64  // Convenience accessor
}

impl HaloNavigationPacket {
    pub fn transmute(bytes: &[u8]) -> Result<Self, &'static str>
    pub fn sog_knots(&self) -> f64
    pub fn cog_degrees(&self) -> f64
}

// Formatting packets to send (in server's info.rs):
pub fn format_heading_packet(heading_deg: f64, counter: u16, timestamp_ms: i64) -> [u8; 72]
pub fn format_navigation_packet(sog_ms: f64, cog_deg: f64, counter: u16, timestamp_ms: i64) -> [u8; 72]
pub fn format_speed_packet(sog_ms: f64, cog_deg: f64) -> [u8; 23]
```

### Address Constants

All multicast addresses are defined once in mayara-core:

```rust
// mayara-core/src/protocol/navico.rs
pub const INFO_ADDR: &str = "236.6.7.8";
pub const INFO_PORT: u16 = 50200;
pub const SPEED_ADDR_A: &str = "236.6.7.5";
pub const SPEED_ADDR_B: &str = "236.6.7.6";
pub const SPEED_PORT_A: u16 = 50201;
pub const SPEED_PORT_B: u16 = 50201;
```

**Key insight:** The server's `navico/info.rs` and `navico/report.rs` import these constants from core, eliminating duplicate address definitions.

---

## Related Documents

- [Architecture Overview](architecture.md)
- [IoProvider Architecture](io-provider.md)
- [Unified Controllers](controllers.md)
