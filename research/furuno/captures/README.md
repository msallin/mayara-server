# Furuno Packet Captures

## drs4dnxt-dual-range-tcp.pcap

TCP command session between TimeZero and a DRS4D-NXT (serial 6424, firmware 01.05)
operating in dual range mode (Range A = 0.5 NM, Range B = 6 NM).

- **Captured from:** Windows machine running TimeZero, via Wireshark
- **Date:** 2026-04-07
- **Radar IP:** 172.31.3.212, port 10100
- **Client IP:** 172.31.3.54
- **Duration:** ~67 seconds
- **Contents:** Range commands ($S62/$N62) for both Range A (drid=0) and Range B (drid=1),
  heartbeat ($NAF), diagnostics ($NF5/$NE3), MainBang auto-adjust ($N83), and a
  periodic status query burst ($R8E, $R8F, $R00 fan status).

Key observations from this capture:
- Range response format confirmed as `$N62,{wire_idx},{unit},{drid}`
- Range B above wire_idx 11 (12 NM) is immediately clamped back by the radar
- $N83 arrives in pairs during dual range (one per range)
- No UDP spoke data in this capture (TCP only); spoke data is multicast on 239.255.0.2:10024
