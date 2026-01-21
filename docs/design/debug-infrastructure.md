# Debug Infrastructure (Dev Mode Only)

> Part of [Mayara Architecture](architecture.md)

This document describes the debug infrastructure for real-time protocol analysis during reverse engineering. It's only available when built with `--features dev` and has zero overhead in production.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DebugHub                                        │
│  - Aggregates events from all DebugIoProviders                              │
│  - Ring buffer (10K events) for history                                      │
│  - Change detection (compares successive radar states)                       │
│  - WebSocket broadcast to debug panel                                        │
└────────────────────────────────┬────────────────────────────────────────────┘
                                 │
       ┌─────────────────────────┼─────────────────────────┐
       │                         │                         │
       ▼                         ▼                         ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│DebugIoProvider   │  │DebugIoProvider   │  │ PassiveListener  │
│(wraps IoProvider)│  │(wraps IoProvider)│  │(multicast only)  │
│                  │  │                  │  │                  │
│ Captures:        │  │ Captures:        │  │ Captures:        │
│ - All send/recv  │  │ - All send/recv  │  │ - Multicast      │
│ - Socket ops     │  │ - Socket ops     │  │   broadcasts     │
│ - Decodes msgs   │  │ - Decodes msgs   │  │ - Chart plotter  │
│                  │  │                  │  │   triggered      │
│   Furuno         │  │   Navico         │  │   state changes  │
└──────────────────┘  └──────────────────┘  └──────────────────┘
         │                     │                      │
         └─────────────────────▼──────────────────────┘
                        TokioIoProvider
```

---

## Key Components

| Component | Location | Purpose |
|-----------|----------|---------|
| `DebugHub` | `debug/hub.rs` | Central event aggregator and broadcaster |
| `DebugIoProvider<T>` | `debug/io_wrapper.rs` | Wrapper that captures all IoProvider traffic |
| `PassiveListener` | `debug/passive_listener.rs` | Listens to multicast for chart plotter effects |
| `ProtocolDecoder` | `debug/decoders/*.rs` | Brand-specific message decoding |
| `ChangeDetector` | `debug/change_detection.rs` | Correlates commands with state changes |
| `DebugRecorder` | `debug/recording.rs` | Records sessions to `.mdbg` files |

---

## Integration Point

In `core_locator.rs`, when `cfg!(feature = "dev")`, the IoProvider can be wrapped:

```rust
#[cfg(feature = "dev")]
let io = DebugIoProvider::new(
    TokioIoProvider::new(...),
    debug_hub.clone(),
    radar_id.clone(),
    brand.to_string(),
);

#[cfg(not(feature = "dev"))]
let io = TokioIoProvider::new(...);
```

---

## Visibility Limitations

| Traffic | Through DebugIoProvider | Through PassiveListener |
|---------|:-----------------------:|:-----------------------:|
| Our commands → radar | Yes | - |
| Radar responses → us | Yes | - |
| Chart plotter → radar | No | No |
| Radar multicast status | Yes | Yes |

For full traffic capture including chart plotter commands, developers should use `tcpdump` alongside the Protocol Debugger.

---

## Related Documents

- [Architecture Overview](architecture.md)
- [Protocol Debugger User Guide](../user-guide/protocol-debugger.md)
