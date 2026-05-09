# Telemetry System Architecture

This document describes the shared architecture across Spencer (sensor source), Abraham (SD logger), and Jacob (LoRa uplink).

## Board Roles

| Board | Primary role | Input | Output |
|---|---|---|---|
| Spencer | Sensor acquisition and packet construction | Onboard sensors + GPS | Framed I2C telemetry |
| Abraham | Persistent storage logger | Framed I2C (SD-destination) | `current-data.bin` + log text |
| Jacob | Radio uplink | Framed I2C (radio-destination) + PPS | LoRa packet (`callsign + telemetry`) |

## Shared Telemetry and Framing Contract

| Contract item | Value |
|---|---|
| Telemetry packet size | 98 bytes |
| I2C frame max size | 32 bytes |
| I2C header size | 2 bytes |
| I2C payload bytes per frame | up to 30 bytes |
| Header byte0 bit7 | Start of telemetry packet |
| Header byte0 bit0 | SD destination |
| Header byte0 bit1 | Radio destination |
| Header byte1 | 8-bit checksum of payload bytes |

All receivers follow the same assembly behavior:

- Start frame resets packet assembly state.
- Continuations are accepted only when a packet is in progress.
- Checksum mismatch invalidates the current partial packet.
- Packet completes only at exactly 98 bytes.

## Inter-Board Data Flow

```text
              +-------------------+
              |  Spencer Board    |
              |  Build 98-byte    |
              |  telemetry packet |
              +---------+---------+
                        |
                 framed I2C broadcast/general-call style
                        |
        +---------------+----------------+
        |                                |
        v                                v
+---------------+                 +---------------+
| Abraham Board |                 | Jacob Board   |
| SD path (bit0)|                 | Radio path    |
| reassemble -> |                 | (bit1)        |
| write binary  |                 | reassemble -> |
| + log events  |                 | PPS-window TX |
+---------------+                 +-------+-------+
                                          |
                                          v
                                    LoRa RF uplink
```

## Runtime Behavior

### Spencer side

- Continuously samples and updates telemetry fields and validity bits.
- Sends SD-destination frames every main loop pass.
- Sends radio-destination frames in a PPS-offset window.

### Abraham side

- Filters for SD-destination frames.
- Reassembles and validates packet stream.
- Persists raw packets to SD; logs operational events and failure reasons.

### Jacob side

- Filters for radio-destination frames.
- Reassembles and validates packet stream.
- Copies complete packet into transmit buffer, prefixes callsign, and sends in scheduled TX window.

## End-to-End Operation Sequence

```text
1) Spencer samples sensors and refreshes GPS fields.
2) Spencer serializes current telemetry snapshot (98 bytes).
3) Spencer emits framed I2C chunks with checksum and destination bits.
4) Abraham accepts SD frames, rebuilds packet, and writes to SD binary log.
5) Jacob accepts radio frames, rebuilds packet, and marks packet_ready.
6) Jacob PPS timing opens TX window and transmits callsign+telemetry over LoRa.
7) Offline/ground tools decode stored 98-byte packets (viewer in Spencer repo).
```

## Operational Constraints and Notes

- Packet size and frame header semantics are hard protocol boundaries; changing them requires coordinated updates in all three firmware projects and host tooling.
- SD and radio paths are intentionally split by destination flags so each receiver can ignore unrelated traffic with minimal overhead.
- The packet viewer in the Spencer repo is the reference decoder for the 98-byte payload layout used by the system.
