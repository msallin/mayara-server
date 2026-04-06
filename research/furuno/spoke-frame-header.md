# Furuno DRS Spoke UDP Frame Header Analysis

Cross-referenced from three sources:
1. **radar.dll disassembly** (radare2 on `\FecDll_x64\radar.dll`) — authoritative, parses raw UDP
2. **Fec.FarApi.dll decompilation** (ilspycmd) — managed wrapper, struct definitions
3. **MaxSea.Radar.dll decompilation** (ilspycmd) — application-level spoke processing

## Frame Structure

Each UDP packet (multicast `239.255.0.2:10024` or broadcast `172.31.255.255:10024`) contains:

```
[16-byte frame header] [spoke_0] [spoke_1] ... [spoke_N]
```

Each spoke within the frame:
```
[4-byte spoke sub-header] [compressed echo data]
```

## Frame Header (bytes 0-15)

### Bytes 0-7: Packet Header

| Byte | Bits | Field | Description |
|------|------|-------|-------------|
| 0 | [7:0] | `packet_type` | **Always 0x02.** Validated by radar.dll; frame is rejected if not 0x02. |
| 1 | [7:0] | `sequence_number` | Packet sequence number. Saved by radar.dll (`var_b9h`) but not used for header parsing. Increments per packet. |
| 2 | [7:0] | `total_length_hi` | High byte of total packet data length. |
| 3 | [7:0] | `total_length_lo` | Low byte. Full length = `(byte[2] << 8) + byte[3]`. |
| 4-7 | 32 | `timestamp` | 32-bit timestamp: `byte[4] + (byte[5] << 8) + (byte[6] << 16) + (byte[7] << 24)` (little-endian). |

### Bytes 8-11: Sweep Metadata

| Byte | Bits | Field | Description |
|------|------|-------|-------------|
| 8 | [7:0] | `spoke_data_len_lo` | Low 8 bits of per-spoke data length (in 4-byte units). |
| 9 | [0] | `spoke_data_len_hi` | Bit 0: high bit of data length. Full length per spoke = `((byte[9] & 0x01) << 8 \| byte[8]) * 4 + 4` bytes (includes the 4-byte per-spoke sub-header). |
| 9 | [7:1] | `spoke_count` | Number of spokes in this frame: `byte[9] >> 1`. Max observed: ~8-20 spokes per packet. |
| 10 | [7:0] | `sample_count_lo` | Low 8 bits of samples-per-spoke count. |
| 11 | [2:0] | `sample_count_hi` | Bits 0-2: high 3 bits. Full count = `(byte[11] & 0x07) << 8 \| byte[10]`. Typical: 883 samples. |
| 11 | [4:3] | `encoding` | Compression type (0-3). See encoding table below. |
| 11 | [5] | `heading_valid` | 1 = heading data in per-spoke headers is valid. 0 = no heading from antenna. |
| 11 | [7:6] | `radar_id` | **Radar identifier (0-3).** In dual range mode: 0 = Range A, 1 = Range B. Used by radar.dll to index into the per-radar sweep buffer: `buffer_base + radar_id * 0x104024`. |

### Bytes 12-15: Range and Status

| Byte | Bits | Field | Description |
|------|------|-------|-------------|
| 12 | [5:0] | `range_index` | Wire index (0-21) for the radar range. Non-sequential mapping to distance. |
| 12 | [7:6] | `range_status` | Range status flags. Exact meaning unknown. |
| 13 | [4:3] | `range_resolution` | Range resolution selector. Used in spoke length calculation within radar.dll. |
| 13 | other | | Other bits of byte 13 also carry range-related metadata. |
| 14 | [7:0] | `range_value_lo` | Low byte of a secondary range value. |
| 15 | [2:0] | `range_value_hi` | High 3 bits. Full value = `(byte[15] & 0x07) << 8 \| byte[14]`. |
| 15 | [3] | `flag_bit3` | Unknown flag (saved in radar.dll as `var_dfh`). |
| 15 | [5:4] | `echo_type` | Echo type indicator. 0 = no secondary echo data, nonzero = secondary echo present. Also related to heading validity — passed as the `hdg_flg` callback parameter. |
| 15 | [7:6] | `status_bits` | Additional status bits. |

## Per-Spoke Sub-Header (4 bytes per spoke)

Starting at frame offset 16, repeated `spoke_count` times:

| Offset | Bits | Field | Description |
|--------|------|-------|-------------|
| 0 | [7:0] | `angle_lo` | Low byte of spoke azimuth angle. |
| 1 | [4:0] | `angle_hi` | High 5 bits. Full angle = `(byte[1] & 0x1F) << 8 \| byte[0]`. Range 0-8191 (13-bit). |
| 1 | [7:5] | | Upper bits — unused or reserved. |
| 2 | [7:0] | `heading_lo` | Low byte of antenna heading. |
| 3 | [4:0] | `heading_hi` | High 5 bits. Full heading = `(byte[3] & 0x1F) << 8 \| byte[2]`. Range 0-8191 (13-bit). |
| 3 | [7:5] | | Upper bits — unused or reserved. |

After the 4-byte sub-header, compressed echo sample data follows. The length depends on
the compression encoding. The total data consumed per spoke (including sub-header) equals the
`spoke_data_len` computed from bytes 8-9.

## Compression Encodings

| Value | Mode | Description |
|-------|------|-------------|
| 0 | Raw | Uncompressed: `memcpy` of `sample_count` bytes directly. |
| 1 | RLE | Run-length encoded. Even bytes (bit 0=0): literal sample value. Odd bytes (bit 0=1): repeat count = `val >> 1` (0 means 128). |
| 2 | RLE+Delta | First spoke in frame: same as encoding 1. Subsequent spokes: differential decode using previous spoke as reference. |
| 3 | Delta | Always differential against previous spoke. 2-bit control: 00=new literal, 01=repeat current, 10=copy from previous spoke, 11=reserved. |

## Callback Delivery

The native radar.dll processes the UDP frame and delivers individual spokes via callback:

```c
void callback(
    int   radarNo,    // from header byte 11 bits [7:6]
    short status,     // transmit status (0=PREHEATING, 1=STDBY, 2=TX)
    byte* echo,       // decompressed echo data (up to 1024 bytes)
    short sweep_len,  // number of valid echo samples
    short scale,      // scale value (from header)
    short range,      // wire index from header byte 12 bits [5:0]
    short angle,      // 13-bit angle from per-spoke sub-header
    short heading,    // 13-bit heading from per-spoke sub-header
    short hdg_flg     // heading valid flag (from header byte 15 bits [5:4] combined with byte 11 bit 5)
);
```

The callback wrapper in Fec.FarApi.dll also:
- Converts heading to degrees: `heading * 0.0439453125` (= 360/8192)
- Detects rotation completion when angle wraps back
- Maps the range wire index through `_rangeGetTbl[]` before delivery

## Comparison with Current Rust Implementation

### What Our Code Gets Right

| Field | Rust Parsing | Correct? |
|-------|-------------|----------|
| `data[0]` == 0x02 | Checked as frame validation | YES |
| spoke_count from `data[9]` | `data[9] >> 1` | YES |
| sample_count from `data[10-11]` | `((data[11] & 0x07) << 8) \| data[10]` | YES |
| encoding from `data[11]` | `(data[11] & 0x18) >> 3` | YES |
| range wire_index from `data[12]` | `data[12] as i32` | **PARTIALLY** — should mask with `0x3F` to isolate bits [5:0]. Currently reads all 8 bits, but in practice the high 2 bits (range_status) are likely 0 for DRS models. |
| per-spoke angle | `(sweep[1] << 8) \| sweep[0]` as u16 | **PARTIALLY** — should mask byte[1] with `0x1F` for 13-bit angle. Works because angles < 8192 fit in 13 bits and upper bits are zero. |
| per-spoke heading | `(sweep[3] << 8) \| sweep[2]` as u16 | **PARTIALLY** — same 13-bit masking issue as angle. |
| Encoding 0-3 decompression | All four encoders implemented | YES |

### What Our Code Gets Wrong or Imprecise

1. **`v1` calculation** (currently `(data[8] + (data[9] & 0x01) * 256) * 4 + 4`): This is the per-spoke data length in bytes, NOT used anywhere after computation. The comment says "range?" which is incorrect — it's the spoke data stride.

2. **`have_heading`** (currently `(data[15] & 0x30) >> 3`): This reads bits [5:4] of byte 15 and shifts right by 3, giving a value 0-6. But per radar.dll, heading validity is primarily at **byte 11 bit 5**. The byte 15 bits [5:4] are `echo_type`, which relates to secondary echo data. Our code works in practice because both fields tend to be nonzero together when heading is present, but the correct field for heading validity is `(data[11] & 0x20) >> 5`.

3. **`radar_no`** (currently `data[13]`): Per the radar.dll disassembly, the radar identifier is at **byte 11 bits [7:6]**, NOT byte 13. Byte 13 carries range resolution metadata. The correct extraction is `(data[11] & 0xC0) >> 6`.

### Unused Fields We Can Now Use

| Byte | Field | Use Case |
|------|-------|----------|
| 11 [7:6] | `radar_id` | **Dual range demuxing.** This is the correct field for Range A (0) vs Range B (1). Currently stored as `v3` but not used. |
| 11 [5] | `heading_valid` | **Correct heading validity flag.** Should replace the current byte 15 extraction. |
| 1 | `sequence_number` | Packet loss detection / reordering. |
| 2-3 | `total_length` | Frame integrity validation. |
| 4-7 | `timestamp` | Spoke timing / latency measurement. |
| 8-9 [0:8] | `spoke_data_len` | Per-spoke data stride, useful for robust frame parsing instead of relying on decompressor consumed-byte count. |
| 12 [7:6] | `range_status` | Unknown, but could indicate range change in progress. |
| 14-15 [2:0,7:0] | `range_value` | Secondary range representation (11-bit), possibly the actual distance. |
| 15 [5:4] | `echo_type` | Secondary echo data presence (e.g., Doppler overlay). |

## Recommended Fixes

1. **Fix `radar_id` extraction**: Change from `data[13]` to `(data[11] & 0xC0) >> 6`. This is our `v3` field that was already being computed but not used!

2. **Fix `have_heading` extraction**: Change from `(data[15] & 0x30) >> 3` to `(data[11] & 0x20) >> 5`. This gives a clean 0/1 boolean.

3. **Mask `range_index`**: Change from `data[12]` to `data[12] & 0x3F` to isolate the 6-bit range wire index.

4. **Mask per-spoke angles**: Apply `& 0x1FFF` to angle and heading values for correct 13-bit extraction.
