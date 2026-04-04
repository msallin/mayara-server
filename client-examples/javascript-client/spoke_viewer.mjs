#!/usr/bin/env node
/**
 * Spoke viewer for mayara-server.
 *
 * Connects to a running mayara-server, discovers the first radar,
 * and displays sampled spoke data as ASCII art.
 *
 * The protobuf definition is read from the source tree so it stays
 * in sync with what the server writes.
 *
 * Usage:
 *   node spoke_viewer.mjs [--url http://localhost:6502]
 *
 * Requirements (installed automatically by run.sh):
 *   npm install protobufjs ws
 */

import { dirname, resolve } from "path";
import { fileURLToPath } from "url";
import { parseArgs } from "util";

import protobuf from "protobufjs";
import WebSocket from "ws";

const __dirname = dirname(fileURLToPath(import.meta.url));

// ---------------------------------------------------------------------------
// Load protobuf from source tree
// ---------------------------------------------------------------------------

const PROTO_PATH = resolve(
  __dirname,
  "..",
  "..",
  "src",
  "lib",
  "protos",
  "RadarMessage.proto"
);

const root = protobuf.loadSync(PROTO_PATH);
const RadarMessage = root.lookupType("RadarMessage");

// ---------------------------------------------------------------------------
// REST helpers
// ---------------------------------------------------------------------------

async function fetchJson(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: ${res.status} ${res.statusText}`);
  return res.json();
}

async function discoverRadar(baseUrl) {
  const data = await fetchJson(
    `${baseUrl}/signalk/v2/api/vessels/self/radars`
  );
  const ids = Object.keys(data);
  if (ids.length === 0) {
    console.error(
      "No radars found. Is the server running with --emulator or a real radar?"
    );
    process.exit(1);
  }
  return { id: ids[0], info: data[ids[0]] };
}

async function fetchCapabilities(baseUrl, radarId) {
  return fetchJson(
    `${baseUrl}/signalk/v2/api/vessels/self/radars/${radarId}/capabilities`
  );
}

// ---------------------------------------------------------------------------
// ASCII art helpers
// ---------------------------------------------------------------------------

const SHADE = " ·:+#@"; // 6 levels

function spokeToAscii(data, width = 64) {
  if (!data || data.length === 0) return " ".repeat(width);
  const step = Math.max(1, data.length / width);
  let chars = "";
  for (let i = 0; i < width; i++) {
    const idx = Math.min(Math.floor(i * step), data.length - 1);
    const val = data[idx];
    const level =
      val > 0
        ? Math.min(SHADE.length - 1, Math.floor((val * SHADE.length) / 64))
        : 0;
    chars += SHADE[level];
  }
  return chars.padEnd(width);
}

function fmtAngle(angle, spokesPerRev) {
  return `${((angle * 360) / spokesPerRev).toFixed(1)}°`.padStart(7);
}

function fmtBearing(bearing, spokesPerRev) {
  if (bearing == null) return "  N/A ";
  return `${((bearing * 360) / spokesPerRev).toFixed(1)}°`.padStart(7);
}

function fmtRange(meters) {
  if (meters >= 1852) return `${(meters / 1852).toFixed(1)} nm`;
  return `${meters} m`;
}

function fmtTime(ms) {
  if (ms == null) return "N/A";
  return new Date(Number(ms)).toISOString().slice(11, 19);
}

function fmtPosition(lat, lon) {
  if (lat == null || lon == null) return "N/A";
  const ns = lat >= 0 ? "N" : "S";
  const ew = lon >= 0 ? "E" : "W";
  return `${Math.abs(lat).toFixed(4)}°${ns} ${Math.abs(lon).toFixed(4)}°${ew}`;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

async function main() {
  const { values } = parseArgs({
    options: { url: { type: "string", default: "http://localhost:6502" } },
  });
  const baseUrl = values.url;
  const wsBase = baseUrl.replace(/^http/, "ws");

  // Discover radar
  const { id: radarId, info: radarInfo } = await discoverRadar(baseUrl);
  const caps = await fetchCapabilities(baseUrl, radarId);

  const spokesPerRev = caps.spokesPerRevolution || 2048;
  const maxSpokeLen = caps.maxSpokeLength || 1024;
  const sampleCount = 32;
  const sampleInterval = Math.floor(spokesPerRev / sampleCount);

  // Header
  console.log();
  console.log("=".repeat(78));
  console.log(`  Radar:   ${radarInfo.name || radarId}`);
  console.log(`  Brand:   ${radarInfo.brand || "?"}`);
  if (radarInfo.model) console.log(`  Model:   ${radarInfo.model}`);
  console.log(`  ID:      ${radarId}`);
  console.log(`  Spokes/rev:  ${spokesPerRev}`);
  console.log(`  Max spoke:   ${maxSpokeLen} samples`);
  console.log("=".repeat(78));
  console.log();
  console.log("Connecting to spoke stream...");

  // Build WebSocket URL, replacing host if needed
  let spokeUrl = radarInfo.spokeDataUrl;
  const spokeHost = new URL(spokeUrl).origin;
  const targetHost = new URL(wsBase).origin.replace(/^http/, "ws");
  spokeUrl = spokeUrl.replace(spokeHost, targetHost);
  console.log(`  URL: ${spokeUrl}`);
  console.log();

  // Collect one revolution of spokes
  const seen = new Set();
  const sampled = [];
  let currentRange = 0;
  let currentTime = null;
  let currentLat = null;
  let currentLon = null;
  let revCount = 0;
  let prevAngle = null;

  const ws = new WebSocket(spokeUrl);
  ws.binaryType = "arraybuffer";

  for await (const [data] of on(ws, "message")) {
    if (typeof data === "string") continue;

    const buf = Buffer.isBuffer(data) ? data : Buffer.from(data);
    const msg = RadarMessage.decode(buf);

    let done = false;
    for (const spoke of msg.spokes) {
      // Detect revolution wrap
      if (prevAngle !== null && spoke.angle < prevAngle) {
        revCount++;
        if (revCount >= 1) {
          done = true;
          break;
        }
      }
      prevAngle = spoke.angle;

      currentRange = spoke.range;
      if (spoke.time != null) currentTime = spoke.time;
      if (spoke.lat != null) currentLat = spoke.lat;
      if (spoke.lon != null) currentLon = spoke.lon;

      // Sample this spoke?
      const bucket = Math.floor(spoke.angle / sampleInterval);
      if (!seen.has(bucket)) {
        seen.add(bucket);
        sampled.push(spoke);
      }
    }

    if (done) break;
  }

  ws.terminate();

  // Sort by angle
  sampled.sort((a, b) => a.angle - b.angle);

  // Print range info
  console.log(`  Range:    ${fmtRange(currentRange)} (${currentRange} m)`);
  console.log(`  Time:     ${fmtTime(currentTime)} UTC`);
  console.log(`  Position: ${fmtPosition(currentLat, currentLon)}`);
  console.log();

  // Print spoke field documentation
  console.log("Spoke fields (from RadarMessage.proto):");
  console.log(
    `  angle   - Rotation from bow [0..${spokesPerRev}) clockwise`
  );
  console.log("  bearing - True bearing from North (optional)");
  console.log("  range   - Range of last pixel in meters");
  console.log("  time    - Epoch milliseconds (optional)");
  console.log("  lat/lon - Radar position (optional)");
  console.log("  data    - Pixel intensity bytes (0 = no return)");
  console.log();

  // Legend
  console.log(
    `  ASCII legend:  ${[...SHADE].map((c, i) => `${c}=${i}`).join("  ")}`
  );
  console.log(
    `                 (pixel values are mapped to ${SHADE.length} levels)`
  );
  console.log();

  // Column headers
  console.log(
    `  ${"angle".padStart(7)} ${"brng".padStart(6)} ${"range".padStart(7)} ${"len".padStart(4)}  spoke data`
  );
  console.log(
    `  ${"─".repeat(7)} ${"─".repeat(6)} ${"─".repeat(7)} ${"─".repeat(4)}  ${"─".repeat(64)}`
  );

  for (const spoke of sampled) {
    const angleStr = fmtAngle(spoke.angle, spokesPerRev);
    const bearing =
      spoke.bearing != null ? spoke.bearing : null;
    const bearingStr = fmtBearing(bearing, spokesPerRev);
    const rangeStr = fmtRange(spoke.range).padStart(7);
    const dataLen = String(spoke.data.length).padStart(4);
    const ascii = spokeToAscii(spoke.data);

    console.log(
      `  ${angleStr} ${bearingStr} ${rangeStr} ${dataLen}  ${ascii}`
    );
  }

  console.log();
  console.log(
    `  Sampled ${sampled.length} of ${spokesPerRev} spokes (1 per ${sampleInterval})`
  );
  console.log();
}

/**
 * Simple async iterator for Node.js EventEmitter events.
 * Yields arrays of event arguments until the emitter errors or closes.
 */
function on(emitter, event) {
  const queue = [];
  let resolve;
  let done = false;

  emitter.on(event, (...args) => {
    if (resolve) {
      const r = resolve;
      resolve = null;
      r({ value: args, done: false });
    } else {
      queue.push(args);
    }
  });
  emitter.on("close", () => {
    done = true;
    if (resolve) resolve({ value: undefined, done: true });
  });
  emitter.on("error", (err) => {
    done = true;
    if (resolve) resolve({ value: undefined, done: true });
  });

  return {
    [Symbol.asyncIterator]() {
      return {
        next() {
          if (queue.length > 0) {
            return Promise.resolve({ value: queue.shift(), done: false });
          }
          if (done) return Promise.resolve({ value: undefined, done: true });
          return new Promise((r) => {
            resolve = r;
          });
        },
      };
    },
  };
}

main().catch((err) => {
  console.error(err.message || err);
  process.exit(1);
});
