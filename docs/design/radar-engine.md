# RadarEngine: Unified Feature Management

> Part of [Mayara Architecture](architecture.md)

This document describes the RadarEngine that provides unified management of radar controllers along with all feature processors.

---

## Overview

The `RadarEngine` in `mayara-core/engine/mod.rs` provides unified management of radar controllers along with all feature processors (ARPA, GuardZones, Trails, DualRange). Both server and WASM use the same RadarEngine, eliminating code duplication for feature management.

---

## RadarEngine Structure

```rust
// mayara-core/src/engine/mod.rs

/// Wrapper around a controller with all its feature processors
pub struct ManagedRadar {
    pub controller: RadarController,  // Enum: Furuno/Navico/Raymarine/Garmin
    pub arpa: ArpaProcessor,          // Target tracking
    pub guard_zones: GuardZoneProcessor,  // Zone alerting
    pub trails: TrailStore,           // Position history
    pub dual_range: Option<DualRangeController>,  // For supported models
}

/// Central engine managing all radars
pub struct RadarEngine {
    radars: BTreeMap<String, ManagedRadar>,
}

impl RadarEngine {
    // Lifecycle
    pub fn add_radar(&mut self, id: &str, brand: Brand, ...) -> Result<()>
    pub fn remove_radar(&mut self, id: &str)
    pub fn poll<I: IoProvider>(&mut self, io: &mut I) -> Vec<EngineEvent>

    // Controls (unified dispatch)
    pub fn set_control(&mut self, id: &str, control: &str, value: &Value) -> Result<()>
    pub fn get_state(&self, id: &str) -> Option<RadarStateV5>
    pub fn get_capabilities(&self, id: &str) -> Option<CapabilityManifest>

    // ARPA targets
    pub fn get_targets(&self, id: &str) -> Vec<ArpaTarget>
    pub fn acquire_target(&mut self, id: &str, bearing: f64, dist: f64) -> Result<u32>
    pub fn cancel_target(&mut self, id: &str, target_id: u32) -> Result<()>

    // Guard zones
    pub fn get_guard_zones(&self, id: &str) -> Vec<GuardZone>
    pub fn set_guard_zone(&mut self, id: &str, zone: GuardZone) -> Result<()>

    // Trails
    pub fn get_trails(&self, id: &str) -> TrailData
    pub fn clear_trails(&mut self, id: &str)
}
```

---

## RadarController Enum

The `RadarController` enum wraps brand-specific controllers, providing a unified interface for the engine:

```rust
pub enum RadarController {
    Furuno(FurunoController),
    Navico(NavicoController),
    Raymarine(RaymarineController),
    Garmin(GarminController),
}
```

---

## Server Integration

The server uses `Arc<RwLock<RadarEngine>>` as shared state:

```rust
// mayara-server/src/web.rs

pub type SharedEngine = Arc<RwLock<RadarEngine>>;

pub struct Web {
    session: Session,
    engine: SharedEngine,  // Single unified engine
}

// HTTP handlers become thin wrappers:
async fn get_targets(State(state): State<Web>, ...) -> Response {
    let engine = state.engine.read().unwrap();
    Json(engine.get_targets(&radar_id)).into_response()
}
```

---

## WASM Integration

The WASM plugin embeds RadarEngine directly:

```rust
// mayara-signalk-wasm/src/radar_provider.rs

pub struct RadarProvider {
    io: WasmIoProvider,
    locator: RadarLocator,
    spoke_receiver: SpokeReceiver,
    engine: RadarEngine,  // Same engine as server!
}

// Methods become one-liners:
pub fn get_targets(&self, radar_id: &str) -> Vec<ArpaTarget> {
    self.engine.get_targets(radar_id)
}
```

---

## Benefits of RadarEngine

| Benefit | Impact |
|---------|--------|
| **Bug fixes in one place** | ARPA/GuardZone/Trail bugs fixed once, works everywhere |
| **Consistent API** | Server and WASM expose identical feature APIs |
| **Reduced duplication** | ~1400 lines removed from server + WASM combined |
| **Easier testing** | Test RadarEngine with mock IoProvider |

---

## Related Documents

- [Architecture Overview](architecture.md)
- [Unified Controllers](controllers.md)
- [IoProvider Architecture](io-provider.md)
