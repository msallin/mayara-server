# Recording and Playback System

> Part of [Mayara Architecture](architecture.md)

This document describes the recording and playback system for capturing and replaying radar data.

---

## Overview

The recording and playback system enables capturing radar data to `.mrr` files and replaying them later. This provides two key capabilities:

1. **Developer testing** - SignalK Radar API consumers can test `render()` functions with consistent recorded data without live radar hardware
2. **Demos/exhibitions** - Playback works standalone without radar connection

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              RECORDING PATH                                  │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                         mayara-server (Rust)                          │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌──────────────────────────┐  │   │
│  │  │Radar Drivers│───►│  Recorder   │───►│  ~/.../recordings/*.mrr  │  │   │
│  │  │(Furuno,etc) │    │             │    └──────────────────────────┘  │   │
│  │  └─────────────┘    └─────────────┘                                  │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                           PLAYBACK PATHS (2 options)                         │
│                                                                              │
│  Option A: Standalone (mayara-server only)                                  │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  mayara-server ─► Player ─► Virtual Radar ─► mayara-gui              │   │
│  │                                                                       │   │
│  │  Good for: demos, exhibitions, testing without SignalK               │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  Option B: SignalK (for radar API consumers)                                │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  .mrr file ─► SignalK Plugin ─► radarApi.register() ─► SignalK       │   │
│  │                    │                                        │         │   │
│  │                    │            binaryStreamManager         │         │   │
│  │                    └───────────────────────────────────────►│         │   │
│  │                                                             ▼         │   │
│  │                                           ┌─────────────────────────┐│   │
│  │                                           │  Any Radar Consumer:   ││   │
│  │                                           │  - mayara-gui          ││   │
│  │                                           │  - OpenCPN (future)    ││   │
│  │                                           │  - SignalK dev testing ││   │
│  │                                           └─────────────────────────┘│   │
│  │                                                                       │   │
│  │  Good for: SignalK developers testing render(), chart plotter devs  │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## .mrr File Format (MaYaRa Radar Recording)

Binary format optimized for efficient seeking and playback:

```
┌──────────────────────────┐
│ Header (256 bytes)       │  magic "MRR1", version, radar metadata
├──────────────────────────┤
│ Capabilities (JSON)      │  length-prefixed JSON (v5 capabilities)
├──────────────────────────┤
│ Initial State (JSON)     │  length-prefixed JSON (controls state)
├──────────────────────────┤
│ Frame 0                  │  timestamp + protobuf RadarMessage + state delta
│ Frame 1                  │
│ ...                      │
├──────────────────────────┤
│ Index (for seeking)      │  array of (timestamp, file_offset)
├──────────────────────────┤
│ Footer (32 bytes)        │  index offset, frame count, duration
└──────────────────────────┘
```

**File sizes:** ~15-30 MB/minute, ~1-2 GB/hour

**Compression strategy:**
- Storage: Uncompressed `.mrr` for fast seeking/playback
- Download: Gzip-compressed `.mrr.gz` for transfer (~95% size reduction)
- Upload: SignalK plugin accepts `.mrr.gz`, auto-decompresses

---

## REST API Endpoints

All at `/v2/api/recordings/`:

### Recording Control

```
GET  /v2/api/recordings/radars          # List available radars to record
POST /v2/api/recordings/record/start    # {radarId, filename?}
POST /v2/api/recordings/record/stop
GET  /v2/api/recordings/record/status
```

### Playback Control

```
POST /v2/api/recordings/playback/load   # {filename}
POST /v2/api/recordings/playback/play
POST /v2/api/recordings/playback/pause
POST /v2/api/recordings/playback/stop
POST /v2/api/recordings/playback/seek   # {timestamp_ms}
PUT  /v2/api/recordings/playback/settings  # {loop?, speed?}
GET  /v2/api/recordings/playback/status
```

### File Management

```
GET    /v2/api/recordings/files              # ?dir=subdir
GET    /v2/api/recordings/files/:filename
DELETE /v2/api/recordings/files/:filename
PUT    /v2/api/recordings/files/:filename    # {newName?, directory?}
POST   /v2/api/recordings/files/upload       # Accepts .mrr or .mrr.gz
GET    /v2/api/recordings/files/:filename/download  # Returns .mrr.gz
```

---

## Virtual Radar Registration

During playback, the player registers as a "virtual radar" that appears in the radar list. Playback radars are identified by their ID prefix `playback-*`:

```rust
// Playback radar is distinguished from real radars
let radar_id = format!("playback-{}", base_name);

// Capabilities include isPlayback flag
let capabilities = Capabilities {
    id: radar_id,
    name: format!("Playback: {}", base_name),
    brand: "Playback",
    model: "Recording",
    isPlayback: true,  // GUI uses this to disable controls
    ...metadata_from_mrr_file
};
```

---

## GUI Playback Mode

The mayara-gui detects playback radars and adjusts its behavior:

```javascript
// api.js
export function isPlaybackRadar(radarId) {
  return radarId && radarId.startsWith('playback-');
}

// control.js - Disable controls for playback
if (isPlaybackRadar(radarId)) {
  container.querySelectorAll('input, select, button').forEach(el => {
    el.disabled = true;
  });
  header.appendChild(span({class: 'playback-badge'}, 'PLAYBACK'));
}
```

---

## SignalK Playback Plugin

The `mayara-server-signalk-playbackrecordings-plugin` is a **self-contained** developer tool that reads `.mrr` files directly (no mayara-server required). It:

1. Parses `.mrr` files using JavaScript port of `file_format.rs`
2. Registers as RadarProvider via SignalK Radar API
3. Emits frames through `binaryStreamManager` at correct timing
4. Provides simple playback UI (upload, play/pause/stop, loop)
5. Links to mayara-gui's `viewer.html` for radar display

**Why separate plugin:**
- Keeps main `mayara-server-signalk-plugin` simple for normal users
- Self-contained for developers (single plugin install)
- No coordination between plugins needed

---

## Implementation Files

| Component | Location | Purpose |
|-----------|----------|---------|
| **file_format.rs** | mayara-server/recording/ | .mrr binary format read/write |
| **recorder.rs** | mayara-server/recording/ | Subscribe to radar, write frames |
| **player.rs** | mayara-server/recording/ | Read frames, emit as virtual radar |
| **manager.rs** | mayara-server/recording/ | File listing, metadata, CRUD |
| **recordings.html/js** | mayara-gui/ | Recording/playback UI |
| **mrr-reader.js** | signalk-playback-plugin/ | JS port of file_format.rs |
| **playback.html** | signalk-playback-plugin/ | Minimal playback control UI |

---

## Related Documents

- [Architecture Overview](architecture.md)
- [External Clients](external-clients.md)
- [Data Flows](data-flows.md)
