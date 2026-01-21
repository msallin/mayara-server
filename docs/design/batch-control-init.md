# Batch Control Initialization

> Part of [Mayara Architecture](architecture.md)

This document describes the batch control initialization system that enables dynamic control discovery.

---

## Overview

The capabilities module provides batch functions to generate all controls for a brand or model, enabling server's `control_factory.rs` to initialize controls without hardcoding lists.

---

## Core Functions (mayara-core/capabilities/controls.rs)

```rust
/// Get base controls that exist on all radars of a brand
pub fn get_base_controls_for_brand(brand: Brand) -> Vec<ControlDefinition> {
    // Returns: power, gain, sea, rain, etc.
}

/// Get all controls for a specific model (base + extended)
pub fn get_all_controls_for_model(brand: Brand, model_name: Option<&str>) -> Vec<ControlDefinition> {
    // Uses models::get_model() to look up model's control list
    // Returns base controls + model-specific extended controls
}
```

---

## Server Builders (mayara-server/control_factory.rs)

```rust
/// Convert core ControlDefinitions to server's Control objects
pub fn build_base_controls_for_brand(brand: Brand) -> HashMap<String, Control> {
    let core_defs = controls::get_base_controls_for_brand(brand);
    core_defs.into_iter()
        .map(|def| (def.id.clone(), build_control(&def)))
        .collect()
}

/// Build all controls for a model
pub fn build_all_controls_for_model(brand: Brand, model_name: Option<&str>) -> HashMap<String, Control>

/// Build only extended controls for a model (when model detected after startup)
pub fn build_extended_controls_for_model(brand: Brand, model_name: &str) -> HashMap<String, Control>
```

---

## Initialization Flow

```
1. Radar discovered (unknown model)
   └── settings.rs calls build_base_controls_for_brand(Brand::Navico)
       └── Core returns base controls: power, gain, sea, rain, range, etc.

2. Model identified via report packet (e.g., "HALO24")
   └── settings.rs calls build_extended_controls_for_model(Brand::Navico, "HALO24")
       └── Core looks up HALO24 in models/navico.rs
       └── Returns: dopplerMode, dopplerSpeed, accentLight, seaState, etc.

3. Controls merged into radar state
   └── API /capabilities reflects all available controls
```

**Key insight:** The model database in `mayara-core/models/` is the single source of truth for which controls exist on each radar model. Adding a control to a model's list automatically makes it available through the API.

---

## Adding a New Feature: The Workflow

### Example: Adding a New Control (e.g., "pulseWidth")

**Step 1: Add control definition (mayara-core)**
```rust
// mayara-core/src/capabilities/controls.rs
pub fn control_pulse_width() -> ControlDefinition {
    ControlDefinition {
        id: "pulseWidth",
        name: "Pulse Width",
        control_type: ControlType::Number,
        min: Some(0.0),
        max: Some(3.0),
        ...
    }
}
```

**Step 2: Add to model capabilities (mayara-core)**
```rust
// mayara-core/src/models/furuno.rs
static CONTROLS_NXT: &[&str] = &[
    "beamSharpening", "dopplerMode", ...,
    "pulseWidth",  // ← Add here
];
```

**Step 3: Add dispatch entry (mayara-core)**
```rust
// mayara-core/src/protocol/furuno/dispatch.rs
pub fn format_control_command(control_id: &str, value: i32, auto: bool) -> Option<String> {
    match control_id {
        ...
        "pulseWidth" => Some(format_pulse_width_command(value)),  // ← Add here
        _ => None,
    }
}
```

**Step 4: Done!**
- Server automatically uses new dispatch entry
- WASM automatically uses new dispatch entry
- GUI automatically shows control (reads from /capabilities)
- No server code changes needed!

---

## Persistent Installation Settings

Some radar controls are **write-only** - they can be sent to the radar but cannot be reliably read back. Examples include Furuno's `autoAcquire` (ARPA), `bearingAlignment`, and `antennaHeight`.

These Installation category controls are persisted using the **Signal K Application Data API**, which is implemented in both mayara-server (`storage.rs`) and Signal K itself. This ensures:
1. GUI code works identically in standalone and Signal K modes
2. Settings survive server restarts
3. Settings are restored to radar on reconnect

### Storage Location (aligned with WASM SignalK plugin)
- API: `/signalk/v1/applicationData/global/@mayara/signalk-radar/1.0.0`
- Files: `~/.local/share/mayara/applicationData/@mayara/signalk-radar/1.0.0.json`

### Storage Schema

The `radars` object is keyed by **unique radar identifier** (`{Brand}-{SerialNumber}`), allowing multiple radars from different brands to be stored in the same file:

```json
{
  "radars": {
    "Furuno-RD003212": {
      "bearingAlignment": -5,
      "antennaHeight": 15,
      "autoAcquire": true
    },
    "Raymarine-Q24C-ABC123": {
      "bearingAlignment": 3,
      "antennaHeight": 8
    },
    "Navico-HALO-XYZ789": {
      "bearingAlignment": 0,
      "antennaHeight": 12
    }
  }
}
```

The unique key is obtained from `capabilities.key` in the REST API, which corresponds to the radar's internal key (e.g., `Furuno-{serial}` or `Navico-{serial}`).

### Persistence Flow

```
User sets bearingAlignment to -5° in GUI
  │
  │  GUI gets capabilities.key = "Furuno-RD003212" (unique identifier)
  │
  ├─► GUI: PUT /radars/radar-2/controls/bearingAlignment {value: -5}
  │         Server sends $S81,-50,0 to radar (tenths of degrees)
  │
  └─► GUI: PUT /signalk/v1/applicationData/global/@mayara/signalk-radar/1.0.0
           Body: {"radars":{"Furuno-RD003212":{"bearingAlignment":-5,...}}}
           (uses capabilities.key, not the REST API id)

On server restart / radar reconnect:
  │
  └─► Server loads 1.0.0.json, looks up settings for radar's key
      Server sends: $S81,-50,0  $S84,0,15,0  $S87,1
      REST API /state reflects restored values
```

### Write-Only Control Pattern

Controls with `wire_hints.write_only = true` in mayara-core indicate that:
- The control can be SET but not reliably READ from hardware
- GUI should persist the value via Application Data API
- Server should restore values on controller connect

### Implementation Files
- `mayara-gui/api.js` - `saveInstallationSetting()` and `getInstallationSettings()`
- `mayara-gui/control.js` - Persists Installation category controls after successful change
- `mayara-server/src/storage.rs` - `load_installation_settings()` for server-side loading
- `mayara-server/src/brand/furuno/report.rs` - `restore_installation_settings()` on model detection
- `mayara-core/src/capabilities/controls.rs` - `write_only: true` in wire_hints

---

## Related Documents

- [Architecture Overview](architecture.md)
- [Unified Controllers](controllers.md)
- [Data Flows](data-flows.md)
