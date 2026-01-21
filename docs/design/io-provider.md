# IoProvider Architecture

> Part of [Mayara Architecture](architecture.md)

This document describes the IoProvider abstraction that enables code sharing between WASM and native server platforms.

---

## Key Insight

Both WASM and Server use the **exact same** radar logic from mayara-core.
The only difference is how sockets are implemented.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           mayara-core                                        │
│                    (Pure Rust, no I/O, WASM-compatible)                      │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                       IoProvider Trait                               │    │
│  │  (mayara-core/io.rs)                                                 │    │
│  │                                                                      │    │
│  │  trait IoProvider {                                                  │    │
│  │      // UDP: create, bind, broadcast, multicast, send, recv, close   │    │
│  │      // TCP: create, connect, send, recv_line, recv_raw, close       │    │
│  │      // Utility: current_time_ms(), debug()                          │    │
│  │  }                                                                   │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                       RadarLocator                                   │    │
│  │  (mayara-core/locator.rs)                                           │    │
│  │                                                                      │    │
│  │  - Multi-brand discovery (Furuno, Navico, Raymarine, Garmin)         │    │
│  │  - Beacon packet construction                                        │    │
│  │  - Multicast group management                                        │    │
│  │  - Radar identification and deduplication                            │    │
│  │                                                                      │    │
│  │  Uses IoProvider for all I/O:                                        │    │
│  │    fn start<I: IoProvider>(&mut self, io: &mut I)                    │    │
│  │    fn poll<I: IoProvider>(&mut self, io: &mut I) -> Vec<Discovery>   │    │
│  │    fn shutdown<I: IoProvider>(&mut self, io: &mut I)                 │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                       ConnectionManager                              │    │
│  │  (mayara-core/connection.rs)                                         │    │
│  │                                                                      │    │
│  │  - ConnectionState enum (Disconnected → Connected → Active)          │    │
│  │  - Exponential backoff logic (1s, 2s, 4s, 8s, max 30s)              │    │
│  │  - Furuno login protocol constants and parsing                       │    │
│  │  - ReceiveSocketType (multicast/broadcast fallback)                  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                       Dispatch Functions                             │    │
│  │  (mayara-core/protocol/furuno/dispatch.rs)                          │    │
│  │                                                                      │    │
│  │  - format_control_command(id, value, auto) → wire command            │    │
│  │  - format_request_command(id) → request command                      │    │
│  │  - parse_control_response(line) → ControlUpdate enum                 │    │
│  │                                                                      │    │
│  │  Controllers call dispatch, not individual format functions!         │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                       Unified Brand Controllers                      │    │
│  │  (mayara-core/controllers/)                                         │    │
│  │                                                                      │    │
│  │  FurunoController   - TCP login + command, uses dispatch functions   │    │
│  │  NavicoController   - UDP multicast, BR24/3G/4G/HALO support        │    │
│  │  RaymarineController - UDP, Quantum (solid-state) / RD (magnetron)  │    │
│  │  GarminController   - UDP multicast, xHD series                     │    │
│  │                                                                      │    │
│  │  All controllers use IoProvider for I/O:                            │    │
│  │    fn poll<I: IoProvider>(&mut self, io: &mut I) -> bool            │    │
│  │    fn set_gain<I: IoProvider>(&mut self, io: &mut I, value, auto)   │    │
│  │    fn shutdown<I: IoProvider>(&mut self, io: &mut I)                │    │
│  │                                                                      │    │
│  │  SAME CODE runs on both server (tokio) and WASM (FFI)!              │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
     ┌────────────────────────────┐    ┌────────────────────────────┐
     │      TokioIoProvider       │    │      WasmIoProvider        │
     │   (mayara-server)          │    │   (mayara-signalk-wasm)    │
     │                            │    │                            │
     │   impl IoProvider for      │    │   impl IoProvider for      │
     │   TokioIoProvider {        │    │   WasmIoProvider {         │
     │     fn udp_create() {      │    │     fn udp_create() {      │
     │       socket2::Socket::new │    │       sk_udp_create()      │
     │       tokio::UdpSocket     │    │     }                      │
     │     }                      │    │     fn udp_send_to() {     │
     │     fn udp_recv_from() {   │    │       sk_udp_send()        │
     │       socket.try_recv_from │    │     }                      │
     │     }                      │    │   }                        │
     │   }                        │    │                            │
     └────────────────────────────┘    └────────────────────────────┘
```

---

## Server's CoreLocatorAdapter

The server wraps mayara-core's sync RadarLocator in an async adapter:

```rust
// mayara-server/src/core_locator.rs

pub struct CoreLocatorAdapter {
    locator: RadarLocator,       // from mayara-core (sync)
    io: TokioIoProvider,         // platform I/O adapter
    discovery_tx: mpsc::Sender<LocatorMessage>,
    poll_interval: Duration,     // default: 100ms
}

impl CoreLocatorAdapter {
    pub async fn run(mut self, subsys: SubsystemHandle) -> Result<...> {
        self.locator.start(&mut self.io);  // Same code as WASM!

        loop {
            select! {
                _ = subsys.on_shutdown_requested() => break,
                _ = poll_timer.tick() => {
                    let discoveries = self.locator.poll(&mut self.io);  // Same!
                    for d in discoveries {
                        self.discovery_tx.send(LocatorMessage::RadarDiscovered(d)).await;
                    }
                }
            }
        }
        self.locator.shutdown(&mut self.io);
    }
}
```

---

## What Gets Shared

| Component | Location | WASM | Server | Notes |
|-----------|----------|:----:|:------:|-------|
| **Protocol parsing** | mayara-core/protocol/ | ✓ | ✓ | Packet encode/decode |
| **Protocol formatting** | mayara-core/protocol/navico.rs | ✓ | ✓ | Heading/SOG/COG packets |
| **Model database** | mayara-core/models/ | ✓ | ✓ | Ranges, capabilities |
| **Control definitions** | mayara-core/capabilities/ | ✓ | ✓ | v5 API schemas |
| **Batch control init** | mayara-core/capabilities/controls.rs | ✓ | ✓ | get_base_*, get_all_* |
| **IoProvider trait** | mayara-core/io.rs | ✓ | ✓ | Socket abstraction |
| **RadarLocator** | mayara-core/locator.rs | ✓ | ✓ | **Same discovery code!** |
| **Unified Controllers** | mayara-core/controllers/ | ✓ | ✓ | **ALL 4 brands!** |
| **ConnectionManager** | mayara-core/connection.rs | ✓ | ✓ | State machine, backoff |
| **Dispatch functions** | mayara-core/protocol/furuno/dispatch.rs | ✓ | ✓ | Control routing |
| **RadarState** | mayara-core/state.rs | ✓ | ✓ | update_from_response() |
| **ARPA** | mayara-core/arpa/ | ✓ | ✓ | Target tracking |
| **Trails** | mayara-core/trails/ | ✓ | ✓ | Position history |
| **Guard zones** | mayara-core/guard_zones/ | ✓ | ✓ | Alerting logic |
| **Web GUI** | mayara-gui/ | ✓ | ✓ | Shared assets |

**What's platform-specific:**
- TokioIoProvider (mayara-server) - wraps tokio sockets
- WasmIoProvider (mayara-signalk-wasm) - wraps SignalK FFI
- Axum web server (mayara-server only)
- Spoke data receivers (async in server, poll-based in WASM)

---

## Related Documents

- [Architecture Overview](architecture.md)
- [Unified Controllers](controllers.md)
- [Deployment Modes](deployment-modes.md)
