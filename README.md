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
- **LoRa (defaults in `JacobBoardCode.ino`)**:
  - **CS** → pin **10**
  - **RST** → pin **2**
  - **DIO0 / IRQ** → pin **3** (`RFM95_INT`) — must reach the SX127x **DIO0** pad so RadioHead gets **TX done**. If this wire is wrong or missing, firmware used to **hang forever** after `rf95.send()`; it now **times out** after a few seconds, prints an error, and calls `setModeIdle()`.

### LoRa “freeze” right after `rf95.send()`

RadioHead blocks in `waitPacketSent()` until the RFM95 **DIO0** interrupt fires (pin **3** on this sketch). If the console prints `rf95.send(), len=...` and then nothing until a timeout message:

1. Confirm **DIO0** on the module is wired to Teensy **pin 3** (not DIO1-only boards used as if they were DIO0).
2. Confirm **SPI** (MOSI/MISO/SCK + CS **10**) and **3.3 V** to the module.
3. Keep USB–UART **RX on pin 1** only; accidentally routing UART onto the LoRa IRQ pin can break interrupts.

### Related repos

- **TelemetryCommon** — shared `TelemetryData.h` / `LoRaTelemetryPayload` layout.  
- **SpencerBoardCode** — telemetry source and `telemetry_packet_viewer.py` for SD logs.
