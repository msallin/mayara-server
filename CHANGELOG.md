# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Sections can be: Added Changed Deprecated Removed Fixed Security.

## [Unreleased]

### Added

- Optional TLS support (`--tls-cert` and `--tls-key` flags) (#36, #37)

### Fixed

- All control values now include timestamps in state broadcasts
- Furuno spoke data sockets retry on failure instead of silently staying dead
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
