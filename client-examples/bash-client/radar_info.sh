#!/usr/bin/env bash
#
# Simple radar info client for mayara-server.
#
# Demonstrates all REST API endpoints using curl and jq.
#
# Usage:
#   ./radar_info.sh [http://localhost:6502]
#
# Requirements: curl, jq

set -euo pipefail

BASE_URL="${1:-http://localhost:6502}"
API="${BASE_URL}/signalk/v2/api/vessels/self/radars"

# Check dependencies
for cmd in curl jq; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Error: $cmd is required but not installed." >&2
        exit 1
    fi
done

# Check server is reachable
if ! curl -sf "${BASE_URL}/signalk" >/dev/null 2>&1; then
    echo "Error: Cannot reach ${BASE_URL}. Is mayara-server running?" >&2
    exit 1
fi

sep() { echo ""; echo "── $1 ──"; }

# ─── Discovery ────────────────────────────────────────────────────────────

sep "GET /signalk (server discovery)"
curl -s "${BASE_URL}/signalk" | jq .

sep "GET /signalk/v2/api/vessels/self/radars (list radars)"
RADARS=$(curl -s "${API}")
echo "$RADARS" | jq .

# Pick the first radar
RADAR_ID=$(echo "$RADARS" | jq -r 'keys[0]')
if [ "$RADAR_ID" = "null" ] || [ -z "$RADAR_ID" ]; then
    echo "No radars found."
    exit 1
fi
echo ""
echo "Using radar: ${RADAR_ID}"

# ─── Interfaces ───────────────────────────────────────────────────────────

sep "GET .../interfaces (network interfaces)"
IFACES=$(curl -s "${API}/interfaces")
if echo "$IFACES" | jq . 2>/dev/null; then
    :
else
    echo "$IFACES"
fi

# ─── Capabilities ─────────────────────────────────────────────────────────

sep "GET .../${RADAR_ID}/capabilities (radar capabilities)"
CAPS=$(curl -s "${API}/${RADAR_ID}/capabilities")
echo "$CAPS" | jq '{
    maxRange, minRange, spokesPerRevolution, maxSpokeLength,
    pixelValues, hasDoppler, hasDualRange, hasDualRadar,
    hasSparseSpokes, noTransmitSectors, stationary,
    supportedRanges,
    controlCount: (.controls | length),
    controlNames: [.controls | keys[]]
}'

# ─── Controls ─────────────────────────────────────────────────────────────

sep "GET .../${RADAR_ID}/controls (all control values)"
curl -s "${API}/${RADAR_ID}/controls" | jq .

sep "GET .../${RADAR_ID}/controls/power (single control)"
curl -s "${API}/${RADAR_ID}/controls/power" | jq .

sep "GET .../${RADAR_ID}/controls/range (single control)"
curl -s "${API}/${RADAR_ID}/controls/range" | jq .

# ─── Set a control ────────────────────────────────────────────────────────

sep "PUT .../${RADAR_ID}/controls/gain (set gain to 50)"
curl -s -X PUT -H 'Content-Type: application/json' \
    -d '{"value": 50}' \
    "${API}/${RADAR_ID}/controls/gain"
echo "(empty response = success)"

sep "GET .../${RADAR_ID}/controls/gain (verify)"
curl -s "${API}/${RADAR_ID}/controls/gain" | jq .

# ─── Targets ──────────────────────────────────────────────────────────────

sep "GET .../${RADAR_ID}/targets (list targets)"
curl -s "${API}/${RADAR_ID}/targets" | jq .

sep "POST .../${RADAR_ID}/targets (acquire target)"
curl -s -X POST -H 'Content-Type: application/json' \
    -d '{"bearing": 0.785, "distance": 2000}' \
    "${API}/${RADAR_ID}/targets" | jq .

# ─── OpenAPI spec ─────────────────────────────────────────────────────────

sep "GET .../resources/openapi.json (API spec summary)"
curl -s "${API}/resources/openapi.json" | jq '{
    openapi, title: .info.title, version: .info.version,
    paths: [.paths | keys[]]
}'

echo ""
echo "Done. Swagger UI available at: ${BASE_URL}/swagger-ui/"
