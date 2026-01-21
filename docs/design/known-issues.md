# Known Issues and Workarounds

> Part of [Mayara Architecture](architecture.md)

This document tracks known issues and their workarounds.

---

## mDNS SignalK Discovery Floods Network (December 2025)

**Problem:** When no `--navigation-address` is specified, mayara defaulted to mDNS discovery for SignalK servers. The `mdns-sd` library sends continuous query packets on all network interfaces, flooding the network with `_signalk-tcp._tcp.local.` queries. This caused severe network congestion (ping timeouts, high CPU) especially in multi-NIC setups where radar and LAN share layer 2.

**Workaround:** mDNS discovery is now disabled by default. The `ConnectionType::Disabled` variant prevents the mDNS daemon from starting when `--navigation-address` is not specified.

**To enable SignalK integration:** Use one of these options:
- `--navigation-address eth0` - mDNS on specific interface
- `--navigation-address tcp:192.168.1.100:3000` - Direct TCP connection
- `--navigation-address udp:192.168.1.100:10110` - UDP NMEA listener

**Future fix:** The mdns-sd library needs rate limiting or the browse loop needs throttling. For now, explicit configuration is required for SignalK integration.

---

## Related Documents

- [Architecture Overview](architecture.md)
