# Jacob Board Code

Jacob is the LoRa uplink board. It receives framed `TelemetryData` from Spencer over **I2C (Wire1, slave 0x08)**, reassembles packets, optionally schedules **RFM95** transmissions in a PPS-aligned window using GPS unix time, and logs diagnostics on **Serial1**.

## Debug guide

### Status LED (Teensy)

| Pin | Behavior |
|-----|------------|
| **7** (`LED_PIN`) | **Double short pulse**: a full length-prefixed telemetry packet was reassembled from I2C (`markTelemetryPacketReady`). **Single long pulse**: LoRa TX finished successfully (`waitPacketSent`). **Toggle** on PPS when a LoRa TX window is scheduled from unix epoch + slot timing. At end of `setup()`, the LED is driven **HIGH** as a “ready” indication. |

### UART — Serial1

Firmware calls `Serial1.begin(115200)` in `setup()`. All `Serial1.print` / `println` diagnostics (I2C assembly, PPS scheduling, LoRa send, optional verbose I2C traces) use this port.

**Teensy 4.x and a 3.3 V USB–UART adapter**

| Adapter wire | Teensy connection |
|--------------|-------------------|
| **GND** | **GND** (common ground) |
| **RX** (USB–UART IC receive) | **Pin 1** — Teensy **TX1** (MCU transmits debug text here) |
| **TX** (USB–UART IC transmit) | **Pin 0** — Teensy **RX1** (optional: MCU input if you send commands) |

Use **115200** baud in your serial terminal. **Do not** connect 5 V TTL UART lines directly to Teensy pins; use 3.3 V logic levels only.

### Other useful pins (reference)

- **PPS**: pin **8** (input) — GPS pulse-per-second for timing.
- **LoRa**: see `JacobBoardCode.ino` for `RFM95_CS`, `RFM95_RST`, `RFM95_INT`.

### Related repos

- **TelemetryCommon** — shared `TelemetryData.h` / `LoRaTelemetryPayload` layout.  
- **SpencerBoardCode** — telemetry source and `telemetry_packet_viewer.py` for SD logs.
