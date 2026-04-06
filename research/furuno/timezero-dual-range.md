# TimeZero DRS Dual Range Analysis

Decompiled from `Fec.FarApi.dll`, `MaxSea.Radar.dll`, and `MaxSea.SensorApi.dll`.

## Architecture Overview

Furuno's dual range is **not a single "enable dual range" command**. The native DLL (`\FecDll_x64\radar.dll`)
treats a single physical antenna as **two logical radars**: `radarNo=0` (Range A) and `radarNo=1` (Range B).
Every control command takes a `radarNo` parameter. The DRS firmware handles the multiplexing internally -
it alternates between the two range settings on successive antenna rotations (or interleaves spokes).

## Enabling Dual Range

### Step 1: Configure Radar Sources

```
RmSetRadarSource(0, hostname, 0)   // index=0, rangeNo=0 → Range A
RmSetRadarSource(1, hostname, 1)   // index=1, rangeNo=1 → Range B
```

- `index` is the logical radar slot (0 or 1)
- `hostname` is the radar's network hostname (8 bytes ASCII, e.g., "DRS4DNXT")
- `rangeNo` is the dual range ID: **0 = Range A, 1 = Range B**

When **not** using dual range (single range mode):
```
RmSetRadarSource(0, hostname, 0)   // Range A only
RmSetRadarSource(1, "",       1)   // No source for B (or same host with rangeNo=0)
```

When TimeZero sets the `RadarHostName` (single radar, no explicit dual range), it does:
```
RmSetRadarSource(0, hostname, 0)
RmSetRadarSource(1, hostname, 1)
```
i.e., same hostname, different rangeNo → dual range is implicitly enabled.

### Step 2: Transmit Control

The `DisableSingleRange1Transmission` flag (default: `true`) controls coupling:

- When `true` and radar type is "DRS": setting `TransmitStatusB` **also sets `TransmitStatusA`**
  to the same value. The antenna cannot independently TX/STBY per range — it transmits for both.
- The actual P/Invoke calls:
  ```
  RmcSetTxStby(0, status, wman, w_send, w_stop)  // Range A
  RmcSetTxStby(1, status, wman, w_send, w_stop)  // Range B (also sets A on DRS)
  ```
- `status`: 1 = STBY, 2 = TX
- `wman`: watchman mode flag
- `w_send` / `w_stop`: watchman timer values

### Step 3: Set Range Per Logical Radar

```
RmcSetRange(0, rangeIndex, unit)   // Set Range A
RmcSetRange(1, rangeIndex, unit)   // Set Range B (independent value)
```

- `rangeIndex` uses the native DLL's own index (NOT the `RadarRanges` enum directly)
- `unit`: distance unit (NM, km, etc.)

#### Range Index Translation Tables

The native DLL uses its own range numbering. Translation between native index and the
`RadarRanges` enum index (0-21) is done via lookup tables:

**`_rangeGetTbl`** — native DLL index → RadarRanges enum index (used when receiving):
```
Native:    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21+
Enum idx:  1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  17  19  20  21  16  18   0
```

**`_rangeSetTbl`** — RadarRanges enum index → native DLL index (used when sending):
```
Enum idx:  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21
Native:   21   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  19  15  20  16  17  18
```

Mapping to NM values (native DLL order):
```
Native idx:  0      1     2    3     4    5    6    7    8    9   10   11   12   13   14
NM value:    0.125  0.25  0.5  0.75  1.0  1.5  2.0  3.0  4.0  6.0  8.0  12.0 16.0 24.0 32.0

Native idx: 15    16    17    18     19    20
NM value:   48.0  72.0  96.0  120.0  36.0  64.0
```

Note: native indices 19 (36 NM) and 20 (64 NM) are out of sequence — they were added later.

## How Spokes Are Returned

### Echo Callback

The native DLL delivers spokes via a callback registered with `RmSetEchoCallbackFunc`:

```csharp
delegate void CallbackFunc(
    int   radarNo,      // 0 = Range A, 1 = Range B
    short status,
    ref S_UDPBUF echo,  // raw echo data buffer
    short sweep_len,    // number of samples in this spoke
    short scale,
    short range,        // native DLL range index (use _rangeGetTbl to convert)
    short angle,        // spoke angle (0-8191 = 0°-360°)
    short heading,      // heading value (raw: multiply by 0.0439453125 for degrees)
    short hdg_flg       // 1 = heading data valid, 0 = no heading
);
```

**The `radarNo` field distinguishes Range A (0) from Range B (1) spokes.** Both ranges arrive
through the same callback, interleaved. Each spoke carries its own `range` value so the receiver
always knows which range the spoke belongs to.

### Batch Sweep API

An alternative high-performance API:
```csharp
int RmGetSweeps(int radarNo, out IntPtr sweepBuffer, out IntPtr sweepMetadataBuffer);
```

Returns multiple spokes at once. The metadata per spoke is:
```csharp
struct RadarSweepMetadata {
    short  angle;       // 0-8191
    ushort heading;
    short  headingFlag;
    short  radarNo;     // 0 or 1
    short  range;       // native DLL range index
    short  scale;
    short  sweepLength;
    short  status;
}
```

### Spoke Processing in TimeZero

`SweepSeriousFactory` maintains **two pre-allocated spoke arrays** (8192 entries each):
- `lI3RF0lsCws` — for `radarNo=0` (Range A)
- `evvRFfSfBaE` — for `radarNo=1` (Range B)

Spoke selection: `((radarNo == 0) ? arrayA : arrayB)[angle]`

The `SweepProvider` dispatches based on `radarNo`:
- `radarNo == 0`: updates Range A echo counters (`NumberOfRange0EchoesReceived/Lost`), sends to radar processor A
- `radarNo == 1`: updates Range B echo counters (`NumberOfRange1EchoesReceived/Lost`), sends to radar processor B

Lost spokes are calculated per revolution: if fewer than 2048 spokes received before the angle wraps,
the difference is counted as lost.

### Connection Status Per Range

```csharp
RmGetConnectionEx(out int status, int radarNumber);  // radarNumber: 0 or 1
```

Before reading TransmitStatus or Range for a logical radar, TimeZero checks that
`RmGetConnectionEx` returns connected status for that specific `radarNumber`.

## Broadcast Mode

TimeZero has a `DrsBroadcastMode` concept with two values:
- **`Fusion`** (default): dual range active, both Range A and B displayed, radar names get `_A`/`_B` suffixes
- **`Range1`**: single range mode, Range B data is not sent/displayed

When `DrsBroadcastMode == Range1`, the `SendRadar` method skips `RadarAntennaId.RadarB` entirely.

For a "two radar product" (two physical antennas, not dual range), both sources use `rangeNo=0`
and `DrsBroadcastMode` is forced to `Range1`.

## Radar Models Supporting Dual Range

From `RadarCapabilitiesTable`:

| Model | DualRange Support |
|-------|-------------------|
| DRS (original) | YES |
| DRS X-Class | YES |
| DRS4D-NXT | YES |
| DRS6A-NXT | YES |
| DRS12A-NXT | YES |
| DRS25A-NXT | YES |
| DRS4DL | NO |
| FAR-15x3 | NO |
| FAR-21x7 | NO |
| FAR-3000 | NO |

## Per-Range Controls

All of these Rmc* commands take `radarNo` (0 or 1) and operate independently per range:

| Function | Native API |
|----------|-----------|
| Range | `RmcSetRange(radarNo, range, unit)` |
| TX/Standby | `RmcSetTxStby(radarNo, status, wman, w_send, w_stop)` |
| Gain | `RmcSetGain(radarNo, ...)` |
| Sea Clutter | `RmcSetACSea(radarNo, ...)` |
| Rain Clutter | `RmcSetACRain(radarNo, ...)` |
| Tune | `RmcSetTune(radarNo, ...)` |
| Pulse Width | per-range via PulseWidth settings keyed by range index |
| Blind Sectors | per-radar via settings |
| Echo Color | `RmcGetEchoColorType(radarNo, ...)` |
| ARPA | `RmaGetArpaInfo(radarNo, ...), RmaGetNextArpaInfo(radarNo, ...)` |

## Wire Protocol (from native radar.dll reverse engineering)

The native `\FecDll_x64\radar.dll` (built from `E:\00_Projects\TZ\nn4\src\tzt\Module\Release\Radar.pdb`)
communicates with the DRS antenna using ASCII command strings sent via `Fnet.dll`'s
`NcSendRadarCommandEx(slot, command_string)`.

### Command Format

```
$<prefix><cmd_id>,<param1>,<param2>,...,<dual_range_id>\r\n
```

The **dual range ID is always the last parameter** in every command. This is how the DRS
antenna knows which range a command applies to.

### Prefix Characters

Derived from the string `"SRNXEO"` indexed by the send mode:

| Index | Char | Meaning |
|-------|------|---------|
| 0     | S    | Set command (client → radar) |
| 1     | R    | Set command (server mode) |
| 2     | N    | Query/Request (no params) |
| 3     | X    | Query variant |
| 4     | E    | Query variant |
| 5     | O    | Query variant |

The mode is determined by `NcIsRadarServer()` for set commands, or hardcoded for queries.

### Command ID Mapping

Command IDs are `0x60 + RadarCommandID` (hex):

| Hex | Dec | RadarCommandID | Purpose |
|-----|-----|---------------|---------|
| 60  | 96  | Mode (0)       | Display mode |
| 61  | 97  | DispMode (1)   | Display mode detail |
| 62  | 98  | Range (2)      | **Range control** |
| 63  | 99  | Gain (3)       | Gain control |
| 64  | 100 | ACSea (4)      | Sea clutter |
| 65  | 101 | ACRain (5)     | Rain clutter |
| 66  | 102 | CustomPictureAll (6) | |
| 67  | 103 | CustomPicture (7) | |
| 68  | 104 | PulseWidth (8) | Pulse width |
| 69  | 105 | TxSTBY (9)     | **Transmit/Standby** |
| 6A  | 106 | SelectTarget (10) | ARPA target select |
| 6B  | 107 | ACQTarget (11) | ARPA acquire |
| 6C  | 108 | CancelTarget (12) | ARPA cancel |
| 6D  | 109 | ARPADispMode (13) | |
| 6E  | 110 | AntennaType (14) | |
| 6F  | 111 | KeyCommand (15) | |
| 75  | 117 | Tune (21)      | Tuning |
| 76  | 118 | TuneIndicator (22) | |
| 77  | 119 | Blind (23)     | Blind sectors |

### Key Command Formats

**Range (0x62):**
```
$S62,<range_index>,<unit>,<dual_range_id>\r\n
```
- `range_index`: native DLL range index (0-20, see translation tables above)
- `unit`: distance unit
- `dual_range_id`: 0 = Range A, 1 = Range B

**TX/Standby (0x69):**
```
$S69,<status>,<wman>,<w_send>,<w_stop>,<?>,<dual_range_id>\r\n
```
- `status`: 1 = Standby, 2 = Transmit
- `wman`: watchman mode
- `w_send`, `w_stop`: watchman timer values

**Query (any command):**
```
$N<cmd_id>\r\n
```
No parameters — just the prefix and command ID.

### Network Routing

The `RmSetRadarSource(index, hostname, rangeNo)` call:
1. Stores the hostname and rangeNo in an internal source table (4 entries × 24 bytes at `0x18007c850`)
2. Calls `NcConnectRadarServerEx()` to establish/update the network connection for that slot
3. The source table entry layout:
   - `+0x00` (dword): active flag
   - `+0x04` (dword): rangeNo (dual range ID: 0 or 1)
   - `+0x08` (8 bytes): hostname
   - `+0x14` (dword): Fnet connection/server index

When sending a command for `radarNo=N`, the resolver (`fcn.18000c730`):
1. Looks up `source_table[N]`
2. Returns the Fnet connection index (for routing to the right host)
3. Returns the rangeNo (appended as the last command parameter)

For dual range on the same antenna, both source entries point to the **same hostname/connection**
but carry different `rangeNo` values. The DRS firmware uses the `dual_range_id` parameter in
each command to know which range context to apply it to.

## Known Limitations (from live DRS4D-NXT testing)

- **Range B maximum is 12 NM**: Setting wire_idx above 11 (12 NM) causes the radar to
  acknowledge the new value but immediately send a second response clamping it back to 11.
- **Range B spokes require activation**: The radar does not send Range B spoke data until
  at least one Range Set command with drid=1 (`$S62,<idx>,<unit>,1`) has been sent. Simply
  querying or setting other per-range controls is not sufficient.
- **UDP spoke header**: The DLL callback's `radarNo` maps to byte 15 bit 6 of the raw UDP
  frame header (0 = Range A, 1 = Range B). Byte 11 bits 6-7 — originally identified as
  `radar_id` from radar.dll disassembly — are always `0b11` on DRS4D-NXT and do NOT indicate
  the range. This contradicts the disassembly; see `spoke-frame-header.md` for discussion.
  Evidence: `research/furuno/captures/drs4dnxt-dual-range-tcp.pcap`.
- **TCP connection isolation**: Each client gets its own TCP session. One client's commands
  and responses are not visible to other connected clients.

## Summary for Implementation

1. **Activate dual range** by sending a Range Set command for Range B: `$S62,<idx>,<unit>,1`.
   The radar will begin interleaving Range B spokes in the UDP stream.
2. **Set range independently** per logical radar with `RmcSetRange(0, ...)` and `RmcSetRange(1, ...)`
3. **Transmit is coupled** on DRS models — toggling TX for either range toggles both
4. **Spokes arrive interleaved** through one callback with `radarNo` identifying the range.
   On the wire, Range A/B is encoded in byte 15 bit 6 of the UDP frame header.
5. **All controls are per-radarNo** — each range has independent gain, clutter, tune, etc.
6. **Wire protocol**: ASCII commands `$S<hex_id>,<params>,<dual_range_id>\r\n` sent via TCP
   — the dual range ID (0 or 1) is the last parameter in every command
7. The DRS antenna firmware handles the physical multiplexing transparently based on the
   dual range ID parameter
