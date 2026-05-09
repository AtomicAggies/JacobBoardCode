# Jacob Board Code

Jacob is the radio uplink board. It receives Spencer telemetry over framed I2C, reassembles and validates each 98-byte packet, prepends a 6-byte callsign, and transmits over LoRa in PPS-timed windows.

## What This Firmware Does

- Receives framed I2C telemetry into a small ISR-safe queue.
- Validates frame checksum and packet assembly rules.
- Builds a `LoRaTransmitPacket` (`CALLSIGN[6] + telemetry[98]`).
- Sends via RH_RF95 when the PPS-aligned transmit window opens.

## Receive and Transmit Flow

```text
I2C receive ISR (Wire.onReceive)
        |
        v
Frame queue (size 8, bounded)
        |
        v
processI2CFrame:
  - destination bit1 (RADIO) filter
  - checksum validate
  - start/continuation sequencing
  - 98-byte reassembly
        |
        v
markTelemetryPacketReady()
        |
        v
PPS-scheduled TX window -> LoRa send (callsign + telemetry)
```

## Key Packet and Framing Numbers

| Item | Value |
|---|---|
| Telemetry payload size | 98 bytes |
| Callsign size | 6 bytes (`KJ5NPP`) |
| LoRa transmit payload | 104 bytes |
| I2C max frame size | 32 bytes |
| I2C header size | 2 bytes |
| I2C payload per frame | up to 30 bytes |
| Frame queue size | 8 |
| Packet receive timeout | 1000 ms |

## I2C Header Flags and Validation

| Header field | Meaning |
|---|---|
| Byte0 bit7 | Start of a new telemetry packet |
| Byte0 bit1 | Frame intended for radio path |
| Byte0 bit0 | SD path bit (ignored here) |
| Byte1 | 8-bit checksum over payload bytes |

Assembly behavior is strict and implementation-oriented:

- Continuation without active packet is dropped.
- New start before 98 bytes discards prior partial packet.
- Checksum mismatch discards active partial packet.
- Overflow or timeout discards partial packet.

## PPS and Epoch Scheduling

Jacob disciplines timing from PPS edges, but window placement is derived from Unix-time epoch phase (not fixed offset per second):

```text
PPS rising edge
  -> handlePPS()
     -> read epoch phase from latest telemetry GPS Unix epoch
     -> compute next slot in global EPOCH cycle
     -> if slot occurs in the upcoming second:
          schedule open/close timers for that slot

Within open window:
  if packet_ready and send_armed:
    send exactly once
```

This keeps radio sends aligned to a global epoch timeline anchored at Unix time 0, while still using PPS as the local high-precision second boundary.

### Epoch Tuning

| Constant | Meaning | Practical guidance |
|---|---|---|
| `EPOCH` | Slot spacing in ms on global Unix timeline | Use `EPOCH >= airtime_ms + guard_ms`; common guard is 10-20 ms |
| `SLOT_SUSTAINER_START` | Slot phase offset in ms within each epoch | Use to separate nodes by phase (`offset = node_index * phase_step`) |
| `TX_READY_WINDOW_MS` | Length of the one-shot send window | Keep small (for deterministic send) but large enough for software jitter |

Example:

- If `EPOCH=1400` and `SLOT_SUSTAINER_START=450`, slot times are at `450, 1850, 3250, ...` ms since Unix epoch start.
- Relative to PPS second boundaries, the slot appears at varying offsets (`450 ms`, then `850 ms`, then `250 ms`, ...), which is expected and desired.

## Operational Notes

- LoRa send path uses `RH_RF95` and blocks until packet is sent.
- Counters track valid packets, invalid packets, ignored frames, checksum failures, and dropped ISR queue frames.
- I2C receive address is configured as `0x08`, while Spencer transmits frames using broadcast/general-call style addressing behavior.
