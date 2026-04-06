# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Sections can be: Added Changed Deprecated Removed Fixed Security.

## [Unreleased]

### Changed

- Emulator loops continuously: boat and targets reverse course when targets leave radar range, then turn back at the starting position (closes #38)
- Web server listens on IPv6 dual-stack socket, accepting both IPv4 and IPv6 connections

### Removed

- Duplicate protobuf.js library copies in `web/imports/` and `web/protobuf/` (only `web/gui/protobuf/` is used)

### Added

- Optional TLS support (`--tls-cert` and `--tls-key` flags) (#36, #37)
- Furuno DRS4W model detection (#48)
- ARPA target tracking documentation (`docs/arpa.md`)
- Furuno range units support: Nautical (NM) and Metric (km) modes with per-model range tables
- Furuno dual range support for NXT models: two independent radar instances (Range A/B) with shared TCP/UDP connections

### Fixed

- Furuno spoke header: radar_id now correctly read from byte 11 bits 6-7 (was incorrectly reading byte 13)
- Furuno spoke header: heading_valid now correctly read from byte 11 bit 5 (was reading byte 15 bits 4-5)
- Furuno spoke header: range wire index masked to 6 bits, angle/heading masked to 13 bits

### Fixed

- Raymarine HD main bang suppression control was missing, causing error in logs (#35)
- Multicast reception on multi-homed interfaces (multiple IPs on one NIC) (#51)
- Furuno range report no longer rejects radars set to km or sm display units
- All control values now include timestamps in state broadcasts
- Button controls (clearTrails, clearTargets) no longer sent in state broadcasts
- Unchanged control values no longer re-broadcast to clients
- Blob detection no longer blocks spoke broadcasting to clients
- Navico doppler lookup used wrong nibble for HighBoth mode
- Spoke pixel validation checks full legend size, not just normal colors
- Spoke pixel validation no longer panics in debug builds, logs error and clamps instead
- GUI shows "DISCONNECTED" when server connection is lost
- GUI standby overlay always shows ON-TIME/TX-TIME
- GUI WebSocket reconnect no longer creates duplicate connections
- Target controls (guard zones, exclusion zones) are writable in replay mode
- GUI target acquire used wrong REST endpoint URL
- Guard zones, exclusion rects, and other target controls returned 400 despite succeeding
- NMEA HDT heading was sent in degrees instead of radians
- NMEA VTG COG was sent in degrees instead of radians
- Slow/stationary targets repeatedly lost and reacquired due to turn rejection using noisy measured speed instead of Kalman-estimated speed
- Targets now require 4 updates before being promoted to tracking and displayed, reducing noise from clutter blobs
- Large vessels no longer produce multiple duplicate tracks; young targets within 100m are merged at each revolution end
- Lost targets deleted after 4 revolutions (was 30s); stationary targets after 10 revolutions
- Blob detection now requires strong return (not medium) and at least 25 pixels to suppress wave/clutter arcs
- Doppler-approaching targets tracked everywhere when doppler_auto_track is enabled, not only inside guard zones
- clearTargets button now actually clears all targets immediately from both backend and GUI
- doppler_auto_track setting is now persisted across restarts
- GUI do_change no longer crashes with TypeError when a control is changed before its state is received from the server
- GUI clears all displayed targets when radar connection is lost
- GUI showed DISCONNECTED in Firefox after server restart due to unnecessary state reset when spoke stream reconnect triggered a redundant state stream reconnect
- Heading extracted from spoke data for GUI when no external heading source available
- Furuno spoke data sockets retry on failure instead of silently staying dead
- Recording file upload from the GUI (add missing `POST /recordings/files/upload` route)
- Accept 0xc2 as valid Navico spoke status for HALO20+ compatibility (#27)
- Move inline `display: none` style to CSS for WebGPU warning element
- `NoSuchRadar` returns 404 (was 500), response includes list of valid radar IDs
- `InvalidControlId` returns 404 (was 400/500)
- Unmatched `/signalk/` paths return 404 with list of all valid API endpoints (generated from OpenAPI spec)
- Empty spoke messages no longer broadcast to WebSocket clients
- Spoke WebSocket stream no longer disconnects on broadcast lag (#31)

### Changed

- WebSocket URLs use `wss://` scheme when TLS is enabled
- Client examples accept `--insecure`/`-k` flag for self-signed certificates (opt-in, no longer default)

## [3.4.0]

### Changed

- **API version bumped to 3.2.0** (Signal K Radar API)
- `GET /signalk/v2/api/vessels/self/radars` returns bare radar map (removed `version`/`radars` wrapper)
- All REST endpoints now return unwrapped responses — no wrapper on any endpoint
- OpenAPI schema: `allowed`, `error`, and `timestamp` fields on ControlValue marked `readOnly`
- OpenAPI schema: renamed `RadarApiV3` to `RadarInfo`, `ArpaTargetApi` to `ArpaTarget`

### Removed

- `RadarsResponse`, `FullSignalKResponse`, and `wrap_response()` — no longer needed

## [3.3.0]

### Added

- `timestamp` field on control values, set whenever a value changes
- `hasSparseSpokes` capability field
- `GET /quit` endpoint for clean server shutdown
- `GET /signalk` endpoint for Signal K service discovery
- `SIGNALK_RADAR_API_VERSION` constant in `Cargo.toml [package.metadata]`, used at compile time
- Docker support: Dockerfile, Dockerfile.ci, docker-compose.yml, GitHub Actions workflow for multi-arch builds (#28)
- Integration test suite: 18 REST API tests and 10 WebSocket stream tests
- `tests/run-integration.sh` to start emulator, run all tests, and stop server
- `make test` now runs full integration tests against the emulator
- Client examples: Python, JavaScript, and Bash (`client-examples/`)
- `CLIENT.md` documenting the client examples
- Server logs PID on startup for test harness
- OpenAPI spec version patched at runtime from `SIGNALK_RADAR_API_VERSION`
- Network IPv4 requirements documented in USAGE.md

### Changed

- **API version bumped to 3.1.0** (Signal K Radar API)
- `GET /` redirects to `/gui/` instead of content-negotiating HTML vs JSON
- `GET /signalk` serves the endpoints JSON (moved from `/`)
- Unwrapped REST responses for consistency:
  - `GET .../interfaces` returns bare `InterfaceApi` instead of `{ version, radars: { interfaces: ... } }`
  - `GET .../{id}/capabilities` returns bare `Capabilities` instead of `{ version, radars: { id: { capabilities: ... } } }`
  - `GET .../{id}/controls` returns bare controls map instead of `{ version, radars: { id: { controls: ... } } }`
  - `GET .../{id}/controls/{control_id}` returns bare `ControlValue` (unchanged, was already unwrapped)
  - `GET .../{id}/targets` returns bare array instead of `{ timestamp, targets: [...] }`
- Only `GET /signalk/v2/api/vessels/self/radars` retains the `{ version, radars }` wrapper
- Renamed `lastChanged` to `timestamp` on control values
- GUI `api.js` and `mayara.js` updated for unwrapped responses
- Updated link to releases page in USAGE.md (#25)

### Removed

- Dead code cleanup

### Fixed

- Emulator `/interfaces` endpoint returns empty JSON instead of error text

## [3.2.0]

### Fixed

#23 - Fix recursive call of error handler

### Added

#21 - Add Garmin support for old Garmin radars (HD, xHD)


## [3.1.0]

First semver version. From here on all additions and fixed will be
logged as github issues.

### Added

- Target tracking

## Versions

[Unreleased]: https://github.com/canboat/canboat/compare/v3.4.0..HEAD
[3.4.0]: https://github.com/canboat/canboat/compare/v3.3.0...v3.4.0
[3.3.0]: https://github.com/canboat/canboat/compare/v3.2.0...v3.3.0
[3.2.0]: https://github.com/canboat/canboat/compare/v3.1.0...v3.2.0
[3.1.0]: https://github.com/canboat/canboat/compare/v3.0.0...v3.1.0
