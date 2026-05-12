#include <Arduino.h>
#include <IntervalTimer.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <TelemetryData.h>
#include <cstddef>
#include <cstring>

// Must match TelemetryData.h; if this fails, Jacob will mis-decode GPS unix.
static_assert(offsetof(TelemetryData, gps) + offsetof(GPSData, unixEpoch) == 28,
              "GPS unixEpoch wire offset must be 28 (see TelemetryData.h)");
static_assert(offsetof(TelemetryData, bmp) == 32,
              "BMP wire offset must be 32 (unixEpoch ends at byte 31)");

// ================= CONFIG =================
#define PPS_PIN   8
#define LED_PIN   7
#define CPU_HZ    600000000ULL

#define TX_CYCLE_MS 1400

#define SLOT_BOOSTER_START 0
#define SLOT_SUSTAINER_START 450
#define SLOT_PAYLOAD_START 800
#define SLOT_PADDING 50
#define SLOT_DURATION        350

// Define which slot we are using for this board
// 0 = booster
// 1 = sustainer
#define SLOT_TYPE 1

#define RF95_FREQ 434.0

#define RFM95_CS 10
#define RFM95_RST 2
#define RFM95_INT 3

/** Max wait for TX-done (DIO0 IRQ); match slot duration so TX window and watchdog align. */
#define LORA_WAIT_PACKET_SENT_TIMEOUT_MS (SLOT_DURATION)
static_assert(LORA_WAIT_PACKET_SENT_TIMEOUT_MS <= 65535,
              "LORA_WAIT_PACKET_SENT_TIMEOUT_MS must fit RadioHead uint16_t timeout");

// Epoch length in milliseconds.
// Recommended range: 1000..10000 ms.
// Choose it from slot planning math:
//   slot_spacing_ms = EPOCH
//   must satisfy slot_spacing_ms >= LoRa airtime_ms + guard_ms
// Example: 180 ms airtime + 10 ms guard => EPOCH >= 190 ms
// (then pick a larger practical value like 1400 ms for sparse traffic).
#define EPOCH 1400

#define CALLSIGN "KJ5NPP"

// Uncomment for heavy Serial1 tracing (hex dumps, queue depth, epoch hints):
// #define JACOB_I2C_TELEMETRY_VERBOSE 1
// Set to 1 for Serial1 hex dumps and layout checks (very chatty — enable only
// while diagnosing; can affect timing / queue draining).
#ifndef JACOB_I2C_TELEMETRY_VERBOSE
#define JACOB_I2C_TELEMETRY_VERBOSE 0
#endif

// RadioHead marks handleInterrupt() protected on some releases; subclass for
// TX-done polling when the DIO0 GPIO edge is missed.
class JacobRH_RF95 : public RH_RF95 {
public:
  JacobRH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin)
      : RH_RF95(slaveSelectPin, interruptPin) {}
  void pollRadioInterrupts() { handleInterrupt(); }
};

JacobRH_RF95 rf95(RFM95_CS, RFM95_INT);

// Adjustable send window (DEFAULT = 40 ms)
uint32_t TX_READY_WINDOW_MS = 40;

// Define the start time of the slot for this board
uint32_t SLOT_START =
  SLOT_TYPE == 0 ? SLOT_BOOSTER_START :
  SLOT_TYPE == 1 ? SLOT_SUSTAINER_START :
  SLOT_TYPE == 2 ? SLOT_PAYLOAD_START : 0;

// I2C framing. Spencer sends framed chunks to this slave (0x08). Destination
// flags in byte 0: this board accepts SD or radio (see bits 0/1).
const uint8_t LORA_CALLSIGN_SIZE = 6;
const uint8_t I2C_RECEIVE_ADDRESS = 0x08;
const uint8_t I2C_FRAME_MAX_SIZE = 32;
const uint8_t I2C_FRAME_HEADER_SIZE = 2;
const uint8_t I2C_FRAME_PAYLOAD_SIZE = I2C_FRAME_MAX_SIZE - I2C_FRAME_HEADER_SIZE;

// I2C frame header byte 0:
// bit 0: frame is intended for the SD-card receiver
// bit 1: frame is intended for the radio/antenna receiver
// bit 7: frame starts a new telemetry packet; clear means continuation
// I2C frame header byte 1 is an 8-bit checksum of the payload bytes that
// follow the header.
const uint8_t I2C_FRAME_DESTINATION_SD = 1 << 0;
const uint8_t I2C_FRAME_DESTINATION_RADIO = 1 << 1;
const uint8_t I2C_FRAME_DESTINATION_SD_OR_RADIO =
    I2C_FRAME_DESTINATION_SD | I2C_FRAME_DESTINATION_RADIO;
const uint8_t I2C_FRAME_START = 1 << 7;
// Spencer sends 3 frames per Jacob burst (76-byte telemetry); bursts can stack
// faster than loop() drains. Undersized queue drops middle frames and START
// resets reassembly — unixEpoch straddles chunk 1/2 so it often reads as 0.
const uint8_t FRAME_QUEUE_SIZE = 32;
/** Single slot: only the latest complete telemetry is retained for LoRa. */
const uint8_t TELEMETRY_RING_SIZE = 1;
const unsigned long PACKET_RECEIVE_TIMEOUT_MS = 1000;

struct I2CFrame {
  uint8_t length;
  uint8_t bytes[I2C_FRAME_MAX_SIZE];
};

// LoRa air frame: callsign + stage + sensor subset (see TelemetryData.h).
struct __attribute__((packed)) LoRaTransmitPacket {
  uint8_t callsign[LORA_CALLSIGN_SIZE];
  uint8_t stage_id[1];
  LoRaTelemetryPayload telemetry;
};

static_assert(sizeof(LoRaTransmitPacket) ==
                  LORA_CALLSIGN_SIZE + 1 + sizeof(LoRaTelemetryPayload),
              "LoRaTransmitPacket size mismatch");

volatile uint8_t frameQueueHead = 0;
volatile uint8_t frameQueueTail = 0;
volatile uint16_t droppedFrameCount = 0;
I2CFrame frameQueue[FRAME_QUEUE_SIZE];

TelemetryData i2cTelemetryBuffer;
uint8_t i2cTelemetryBytesReceived = 0;
bool receivingPacket = false;
unsigned long lastPacketFrameMillis = 0;

uint8_t *i2cTelemetryBytes() {
  return reinterpret_cast<uint8_t *>(&i2cTelemetryBuffer);
}

volatile LoRaTransmitPacket lora_tx_buffer;
volatile bool packet_ready = false;

uint8_t expectedTelemetryBytes = 0;

// Latest complete I2C telemetry only (older packets discarded on each commit).
// LoRa TX copies from telemetryRing[telemetryRingNewestIdx] at send.
TelemetryData telemetryRing[TELEMETRY_RING_SIZE];
uint8_t telemetryRingNewestIdx = 0;

uint32_t validPacketCount = 0;
uint32_t invalidPacketCount = 0;
uint32_t ignoredFrameCount = 0;
uint32_t checksumFailureCount = 0;
bool loraReady = false;
IntervalTimer txWindowOpenTimer;
IntervalTimer txWindowCloseTimer;
volatile bool txWindowOpen = false;
volatile bool txWindowSendArmed = false;
uint32_t latestTelemetryUnixEpochSeconds = 0;
/** True when latestTelemetryUnixEpochSeconds is from a full packet, GPS valid bit set, and in range. */
bool hasTelemetryUnixEpochSeconds = false;

// ========== 64-bit cycle counter ==========
volatile uint32_t last_cycle_low = 0;
volatile uint64_t cycle_high = 0;

uint64_t getCycles64() {
  uint32_t low = ARM_DWT_CYCCNT;

  if (low < last_cycle_low) {
    cycle_high += (1ULL << 32);
  }

  last_cycle_low = low;
  return (cycle_high | low);
}

// ========== PPS STATE ==========
volatile uint64_t last_pps_cycles = 0;
volatile uint64_t current_pps_cycles = 0;
volatile bool pps_flag = false;

// ========== CLOCK DISCIPLINE ==========
double freq_correction = 0.0;
double phase_correction = 0.0;

const double Kf = 1e-12;
const double Kp = 1e-3;

// ========== UTC ==========
// Wall-clock seconds from GPS when valid; not advanced on PPS unless we have a valid unix snapshot.
uint64_t utc_seconds = 0;

// ========== TX CYCLE SYNC ==========
volatile uint64_t cycle_start_cycles = 0;

// ========== ENABLE COUNTER ==========
void enableCycleCounter() {
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

// ========== PPS ISR ==========
void pps_isr() {
  current_pps_cycles = getCycles64();
  pps_flag = true;
}

void onTxWindowOpen() {
  txWindowOpenTimer.end();
  txWindowOpen = true;
  txWindowSendArmed = true;
}

void onTxWindowClose() {
  txWindowCloseTimer.end();
  txWindowOpen = false;
  txWindowSendArmed = false;
}

// ========== HANDLE PPS ==========
void handlePPS(uint64_t now) {
  cycle_start_cycles = now;  // Anchor TX cycle to PPS

  if (last_pps_cycles != 0) {
    uint64_t delta = now - last_pps_cycles;

    double expected = (double)CPU_HZ;
    double error = (double)delta - expected;

    freq_correction += Kf * error;
    phase_correction += Kp * error;

    Serial1.print("PPS delta: ");
    Serial1.print((uint32_t)delta);
    Serial1.print(" error: ");
    Serial1.println(error);
  }

  last_pps_cycles = now;

  if (hasTelemetryUnixEpochSeconds) {
    utc_seconds = latestTelemetryUnixEpochSeconds;
  }

  txWindowOpen = false;
  txWindowSendArmed = false;
  txWindowOpenTimer.end();
  txWindowCloseTimer.end();

  if (!hasTelemetryUnixEpochSeconds || EPOCH == 0) {
    Serial1.println(
        "PPS: skip LoRa slot scheduling (no valid GPS unix epoch flag/range or "
        "EPOCH=0)");
    return;
  }
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  uint64_t unixMsAtPps = (uint64_t)latestTelemetryUnixEpochSeconds * 1000ULL;
  uint32_t epochPhaseMs = (uint32_t)(unixMsAtPps % EPOCH);
  uint32_t slotPhaseMs = (uint32_t)(SLOT_START % EPOCH);
  uint32_t timeToSlotMs = 0;
  if (epochPhaseMs <= slotPhaseMs) {
    timeToSlotMs = slotPhaseMs - epochPhaseMs;
  } else {
    timeToSlotMs = EPOCH - (epochPhaseMs - slotPhaseMs);
  }

  // timeToSlotMs is in [0, EPOCH); must allow scheduling up to almost EPOCH ms
  // (EPOCH may exceed 1000 ms — the old >= 1000 check broke TDM for EPOCH=1400).

  Serial1.print("PPS: scheduling LoRa TX window in ");
  Serial1.print(timeToSlotMs);
  Serial1.print(" ms (epoch phase ");
  Serial1.print(epochPhaseMs);
  Serial1.print(" ms, slot ");
  Serial1.print(SLOT_START);
  Serial1.print(" ms, unix=");
  Serial1.print(latestTelemetryUnixEpochSeconds);
  Serial1.println(")");

  txWindowOpenTimer.begin(onTxWindowOpen, timeToSlotMs * 1000ul);
  const uint32_t closeDelayUs = (timeToSlotMs + TX_READY_WINDOW_MS) * 1000ul;
  txWindowCloseTimer.begin(onTxWindowClose, closeDelayUs);
}

// ========== TIME SINCE PPS ==========
double getCorrectedSeconds() {
  uint64_t now_cycles = getCycles64();

  uint64_t pps_cycles_snapshot;
  noInterrupts();
  pps_cycles_snapshot = last_pps_cycles;
  interrupts();

  uint64_t delta_cycles = now_cycles - pps_cycles_snapshot;

  double corrected =
      (double)delta_cycles * (1.0 + freq_correction) + phase_correction;

  return corrected / CPU_HZ;
}

// ========== TX CYCLE TIME ==========
uint32_t getCycleTimeMs() {
  uint64_t now = getCycles64();
  uint64_t delta = now - cycle_start_cycles;

  double corrected =
      (double)delta * (1.0 + freq_correction) + phase_correction;

  double ms = (corrected / CPU_HZ) * 1000.0;

  return ((uint32_t)ms) % TX_CYCLE_MS;
}

// LED: double short pulse = full I2C telemetry packet assembled; long pulse = LoRa TX done.
void ledPulseI2cTelemetryComplete() {
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(90000);
  digitalWrite(LED_PIN, LOW);
  delayMicroseconds(70000);
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(90000);
  digitalWrite(LED_PIN, LOW);
}

void ledPulseLoRaTransmit() {
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(280000);
  digitalWrite(LED_PIN, LOW);
}

// ========== I2C PACKET ASSEMBLY ==========
uint8_t checksumI2CPayload(const uint8_t *payload, uint8_t payloadSize) {
  uint8_t checksum = 0;
  for (uint8_t index = 0; index < payloadSize; index++) {
    checksum += payload[index];
  }
  return checksum;
}

/** GPS unix from a full wire record: VALIDITY_GPS_UNIX_EPOCH, range, memcpy epoch. */
static constexpr uint32_t kGpsUnixMinPlausible = 946684800UL;   // 2000-01-01 UTC
static constexpr uint32_t kGpsUnixMaxPlausible = 4102444800UL;  // ~2099

bool gpsUnixFromWireRecord(const uint8_t *wire, uint8_t wireBytes,
                           uint32_t *outUnix) {
  if (wireBytes < offsetof(TelemetryData, bmp) || outUnix == nullptr) {
    return false;
  }
  if ((wire[3] & VALIDITY_GPS_UNIX_EPOCH) == 0) {
    return false;
  }
  constexpr size_t kUnixOff =
      offsetof(TelemetryData, gps) + offsetof(GPSData, unixEpoch);
  uint32_t unixEpoch = 0;
  memcpy(&unixEpoch, wire + kUnixOff, sizeof(uint32_t));
  if (unixEpoch < kGpsUnixMinPlausible || unixEpoch > kGpsUnixMaxPlausible) {
    return false;
  }
  *outUnix = unixEpoch;
  return true;
}

#if JACOB_I2C_TELEMETRY_VERBOSE
void logVerboseAssembledTelemetry(const uint8_t *wire, uint8_t wireBytes,
                                  uint32_t unixFromStruct) {
  constexpr size_t kUnixOff =
      offsetof(TelemetryData, gps) + offsetof(GPSData, unixEpoch);
  uint32_t unixFromBytes = 0;
  if (wireBytes >= kUnixOff + sizeof(uint32_t)) {
    memcpy(&unixFromBytes, wire + kUnixOff, sizeof(uint32_t));
  }

  Serial1.println("[I2C dbg] --- assembled packet dump ---");
  Serial1.print("[I2C dbg] wireBytes=");
  Serial1.print(wireBytes);
  Serial1.print(" unix@");
  Serial1.print((unsigned)kUnixOff);
  Serial1.print(" struct=");
  Serial1.print(unixFromStruct);
  Serial1.print(" memcpy=");
  Serial1.print(unixFromBytes);
  if (unixFromStruct != unixFromBytes) {
    Serial1.print(" MISMATCH");
  }
  Serial1.println();

  if (wireBytes >= 4) {
    uint16_t ctr = (uint16_t)wire[1] | ((uint16_t)wire[2] << 8);
    Serial1.print("[I2C dbg] pkt ctr=");
    Serial1.print(ctr);
    Serial1.print(" validity=0x");
    Serial1.println(wire[3], HEX);
  }
  if (wireBytes >= offsetof(TelemetryData, gps) + sizeof(int32_t)) {
    int32_t lat = 0;
    memcpy(&lat,
           wire + offsetof(TelemetryData, gps) + offsetof(GPSData, latitude),
           sizeof(lat));
    Serial1.print("[I2C dbg] lat(1e7)=");
    Serial1.println(lat);
  }

  Serial1.print("[I2C dbg] hex[0..15]: ");
  for (size_t i = 0; i < 16 && i < wireBytes; i++) {
    if (i) Serial1.print(' ');
    if (wire[i] < 16) Serial1.print('0');
    Serial1.print(wire[i], HEX);
  }
  Serial1.println();
  Serial1.print("[I2C dbg] hex[24..39] (NED end + unix + BMP start): ");
  for (size_t i = 24; i < 40 && i < wireBytes; i++) {
    if (i > 24) Serial1.print(' ');
    if (wire[i] < 16) Serial1.print('0');
    Serial1.print(wire[i], HEX);
  }
  Serial1.println();
  Serial1.println("[I2C dbg] --- end dump ---");
}
#endif

void discardPartialPacket(const char *reason) {
  if (receivingPacket || i2cTelemetryBytesReceived > 0) {
    invalidPacketCount++;
    Serial1.print("Discarded partial telemetry packet (");
    Serial1.print(i2cTelemetryBytesReceived);
    Serial1.print("/");
    Serial1.print(TELEMETRY_PACKET_MAX_BYTES);
    Serial1.print(" bytes): ");
    Serial1.println(reason);
#if JACOB_I2C_TELEMETRY_VERBOSE
    if (strstr(reason, "new telemetry packet started") != nullptr &&
        i2cTelemetryBytesReceived > 0 && i2cTelemetryBytesReceived < 76) {
      Serial1.println(
          "[I2C dbg] hint: unixEpoch bytes 28-31 span I2C chunk1/chunk2; a "
          "START here often means queue overflow or slow loop vs Spencer rate.");
    }
#endif
  }

  i2cTelemetryBytesReceived = 0;
  expectedTelemetryBytes = 0;
  receivingPacket = false;
  lastPacketFrameMillis = 0;
}

void markTelemetryPacketReady(uint8_t wireBytes) {
  memset(&telemetryRing[0], 0, sizeof(TelemetryData));
  memcpy(&telemetryRing[0], &i2cTelemetryBuffer, wireBytes);
  telemetryRingNewestIdx = 0;

  const uint8_t *wire = i2cTelemetryBytes();
  uint32_t validatedUnix = 0;
  const bool unixOk = gpsUnixFromWireRecord(wire, wireBytes, &validatedUnix);

  if (unixOk) {
    latestTelemetryUnixEpochSeconds = validatedUnix;
    hasTelemetryUnixEpochSeconds = true;
  } else {
    hasTelemetryUnixEpochSeconds = false;
  }

#if JACOB_I2C_TELEMETRY_VERBOSE
  if (wireBytes >= offsetof(TelemetryData, bmp)) {
    logVerboseAssembledTelemetry(wire, wireBytes,
                                 i2cTelemetryBuffer.gps.unixEpoch);
  }
#endif

  noInterrupts();
  if (wireBytes >= TELEMETRY_WIRE_LENGTH_MIN_FOR_LORA) {
    packet_ready = true;
  }
  interrupts();

  validPacketCount++;
  Serial1.print("[I2C] Telemetry packet #");
  Serial1.print(validPacketCount);
  Serial1.print(" assembled, wireLength=");
  Serial1.print(wireBytes);
  Serial1.print(" bytes, GPS unix=");
  if (wireBytes >= offsetof(TelemetryData, bmp)) {
    constexpr size_t kUnixOff =
        offsetof(TelemetryData, gps) + offsetof(GPSData, unixEpoch);
    uint32_t rawUnix = 0;
    memcpy(&rawUnix, wire + kUnixOff, sizeof(uint32_t));
    Serial1.print(rawUnix);
    if (!unixOk) {
      Serial1.print(" (not valid for PPS:");
      if ((wire[3] & VALIDITY_GPS_UNIX_EPOCH) == 0) {
        Serial1.print(" validity=0x");
        Serial1.print(wire[3], HEX);
        Serial1.print(" lacks VALIDITY_GPS_UNIX_EPOCH");
      }
      if (rawUnix < kGpsUnixMinPlausible || rawUnix > kGpsUnixMaxPlausible) {
        Serial1.print(" unix out of range");
      }
      Serial1.print(')');
    }
  } else {
    Serial1.print("(n/a)");
  }
  Serial1.print(", LoRa packet_ready=");
  Serial1.println(wireBytes >= TELEMETRY_WIRE_LENGTH_MIN_FOR_LORA ? "yes" : "no (short packet)");

  ledPulseI2cTelemetryComplete();
}

void processI2CFrame(const I2CFrame &frame) {
  if (frame.length < I2C_FRAME_HEADER_SIZE) {
    Serial1.print("Discarded invalid I2C frame shorter than header: ");
    Serial1.println(frame.length);
    discardPartialPacket("short I2C frame");
    return;
  }

  uint8_t frameFlags = frame.bytes[0];
  uint8_t receivedChecksum = frame.bytes[1];
  uint8_t payloadSize = frame.length - I2C_FRAME_HEADER_SIZE;
  const uint8_t *payload = frame.bytes + I2C_FRAME_HEADER_SIZE;
  bool isStartFrame = (frameFlags & I2C_FRAME_START) != 0;

  if ((frameFlags & I2C_FRAME_DESTINATION_SD_OR_RADIO) == 0) {
    ignoredFrameCount++;
    return;
  }

  uint8_t calculatedChecksum = checksumI2CPayload(payload, payloadSize);
  if (calculatedChecksum != receivedChecksum) {
    checksumFailureCount++;
    Serial1.print("Checksum failure on I2C frame (received 0x");
    Serial1.print(receivedChecksum, HEX);
    Serial1.print(", calculated 0x");
    Serial1.print(calculatedChecksum, HEX);
    Serial1.println(")");
    discardPartialPacket("I2C frame checksum failure");
    return;
  }

  if (isStartFrame) {
    discardPartialPacket("new telemetry packet started before previous packet was complete");
    i2cTelemetryBytesReceived = 0;
    expectedTelemetryBytes = 0;
    receivingPacket = true;
    lastPacketFrameMillis = millis();
    Serial1.print("[I2C] START frame, flags=0x");
    Serial1.println(frameFlags, HEX);
  } else if (!receivingPacket) {
    Serial1.println("Discarded continuation I2C frame with no active telemetry packet");
    return;
  }

  if (payloadSize == 0) {
    discardPartialPacket("empty I2C frame payload");
    return;
  }

  if (i2cTelemetryBytesReceived + payloadSize > TELEMETRY_PACKET_MAX_BYTES) {
    discardPartialPacket("I2C frame would overflow telemetry buffer");
    return;
  }

  memcpy(i2cTelemetryBytes() + i2cTelemetryBytesReceived, payload, payloadSize);
  i2cTelemetryBytesReceived += payloadSize;
  lastPacketFrameMillis = millis();

  {
    static uint32_t lastChunkLogMs = 0;
    uint32_t now = millis();
    if (now - lastChunkLogMs >= 200) {
      lastChunkLogMs = now;
      Serial1.print("[I2C] chunk payload=");
      Serial1.print(payloadSize);
      Serial1.print(" B, assembled=");
      Serial1.print(i2cTelemetryBytesReceived);
      Serial1.print("/");
      if (expectedTelemetryBytes > 0) {
        Serial1.println(expectedTelemetryBytes);
      } else {
        Serial1.println("?");
      }
    }
  }

  if (i2cTelemetryBytesReceived >= 1 && expectedTelemetryBytes == 0) {
    expectedTelemetryBytes = i2cTelemetryBytes()[0];
    if (expectedTelemetryBytes < 2 ||
        expectedTelemetryBytes > TELEMETRY_PACKET_MAX_BYTES) {
      discardPartialPacket("invalid wireLength in telemetry header");
      return;
    }
    Serial1.print("[I2C] wireLength (from header) = ");
    Serial1.println(expectedTelemetryBytes);
  }

  if (expectedTelemetryBytes > 0 &&
      i2cTelemetryBytesReceived > expectedTelemetryBytes) {
    discardPartialPacket("received more bytes than wireLength");
    return;
  }

  if (expectedTelemetryBytes > 0 &&
      i2cTelemetryBytesReceived == expectedTelemetryBytes) {
    markTelemetryPacketReady(expectedTelemetryBytes);
    i2cTelemetryBytesReceived = 0;
    expectedTelemetryBytes = 0;
    receivingPacket = false;
    lastPacketFrameMillis = 0;
  }
}

bool popQueuedFrame(I2CFrame &frame) {
  noInterrupts();
  if (frameQueueHead == frameQueueTail) {
    interrupts();
    return false;
  }

  frame = frameQueue[frameQueueTail];
  frameQueueTail = (frameQueueTail + 1) % FRAME_QUEUE_SIZE;
  interrupts();
  return true;
}

/**
 * Latest-wins (LIFO policy on the backlog): drop queued frames older than the
 * newest START frame so we reassemble only the current Spencer burst, not stale
 * chunks that would otherwise be processed first-in-first-out.
 */
static void skipQueuedI2cToLatestStart() {
  noInterrupts();
  const uint8_t tail = frameQueueTail;
  const uint8_t head = frameQueueHead;
  if (tail == head) {
    interrupts();
    return;
  }

  const uint8_t depth =
      (uint8_t)((head + FRAME_QUEUE_SIZE - tail) % FRAME_QUEUE_SIZE);
  int16_t foundIdx = -1;
  for (uint8_t k = 0; k < depth; k++) {
    const uint8_t idx =
        (uint8_t)((tail + depth - 1 - k + FRAME_QUEUE_SIZE) % FRAME_QUEUE_SIZE);
    const I2CFrame &f = frameQueue[idx];
    if (f.length < I2C_FRAME_HEADER_SIZE) {
      continue;
    }
    if ((f.bytes[0] & I2C_FRAME_START) != 0) {
      foundIdx = idx;
      break;
    }
  }

  if (foundIdx < 0) {
    frameQueueTail = frameQueueHead;
    interrupts();
    discardPartialPacket(
        "queued I2C had no START frame; dropped all (latest-wins)");
    return;
  }

  if ((uint8_t)foundIdx != tail) {
    frameQueueTail = (uint8_t)foundIdx;
    interrupts();
    discardPartialPacket(
        "skipped older queued I2C in favor of latest START (latest-wins)");
  } else {
    interrupts();
  }
}

// ========== RECEIVE I2C FRAME ISR ==========
void receiveI2C(int count) {
  if (count > I2C_FRAME_MAX_SIZE) {
    while (Wire1.available()) {
      Wire1.read();
    }
    droppedFrameCount++;
    return;
  }

  uint8_t nextHead = (frameQueueHead + 1) % FRAME_QUEUE_SIZE;
  if (nextHead == frameQueueTail) {
    while (Wire1.available()) {
      Wire1.read();
    }
    droppedFrameCount++;
    return;
  }

  I2CFrame &frame = frameQueue[frameQueueHead];
  frame.length = 0;
  bool acceptFrame = false;
  while (Wire1.available()) {
    uint8_t byteValue = Wire1.read();
    if (frame.length < I2C_FRAME_MAX_SIZE) {
      frame.bytes[frame.length++] = byteValue;
      if (frame.length == 1) {
        acceptFrame =
            (byteValue & I2C_FRAME_DESTINATION_SD_OR_RADIO) != 0;
      }
    } else {
      droppedFrameCount++;
    }
  }

  if (frame.length < I2C_FRAME_HEADER_SIZE) {
    return;
  }

  if (!acceptFrame) {
    ignoredFrameCount++;
    return;
  }

  frameQueueHead = nextHead;

#if JACOB_I2C_TELEMETRY_VERBOSE
  {
    uint8_t depth =
        (FRAME_QUEUE_SIZE + frameQueueHead - frameQueueTail) % FRAME_QUEUE_SIZE;
    if (depth >= FRAME_QUEUE_SIZE - 4) {
      Serial1.print("[I2C dbg] frame queue depth=");
      Serial1.print(depth);
      Serial1.print("/");
      Serial1.print(FRAME_QUEUE_SIZE - 1);
      Serial1.println(" (near full — risk of drops / split packets)");
    }
  }
#endif
}

void processQueuedI2CFrames() {
  static uint16_t lastDroppedFrameCount = 0;

  noInterrupts();
  uint16_t currentDroppedFrameCount = droppedFrameCount;
  interrupts();

  if (currentDroppedFrameCount != lastDroppedFrameCount) {
    uint16_t droppedSinceLastLog = currentDroppedFrameCount - lastDroppedFrameCount;
    lastDroppedFrameCount = currentDroppedFrameCount;
    Serial1.print("WARNING: I2C frame queue overflow dropped ");
    Serial1.print(droppedSinceLastLog);
    Serial1.println(" frame(s)");
    discardPartialPacket("I2C frame queue overflow");
  }

  skipQueuedI2cToLatestStart();

  I2CFrame frame;
  while (popQueuedFrame(frame)) {
    processI2CFrame(frame);
  }

  if (receivingPacket && lastPacketFrameMillis != 0 &&
      millis() - lastPacketFrameMillis > PACKET_RECEIVE_TIMEOUT_MS) {
    discardPartialPacket("timed out before full telemetry packet received");
  }
}

// ========== LORA SPI / register diagnostics (RadioHead-compatible framing) ==========
static uint8_t loraSpiReadReg8(uint8_t reg) {
  uint8_t v;
  noInterrupts();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(RFM95_CS, LOW);
  SPI.transfer(static_cast<uint8_t>(reg & ~RH_SPI_WRITE_MASK));
  v = SPI.transfer(0);
  digitalWrite(RFM95_CS, HIGH);
  SPI.endTransaction();
  interrupts();
  return v;
}

/** Log chip ID, mode, IRQ flags, and DIO0 pin level (SPI OK usually implies VERSION 0x12). */
static void logLoRaRadioSnapshot(const char *reason) {
  uint8_t ver = loraSpiReadReg8(RH_RF95_REG_42_VERSION);
  uint8_t op = loraSpiReadReg8(RH_RF95_REG_01_OP_MODE);
  uint8_t irq = loraSpiReadReg8(RH_RF95_REG_12_IRQ_FLAGS);
  int dio0 = digitalRead(RFM95_INT);

  Serial1.print("[LoRa dbg] ");
  Serial1.print(reason);
  Serial1.print(" VERSION=0x");
  if (ver < 16) {
    Serial1.print('0');
  }
  Serial1.print(ver, HEX);
  Serial1.print(" OP_MODE=0x");
  if (op < 16) {
    Serial1.print('0');
  }
  Serial1.print(op, HEX);
  Serial1.print(" IRQ_FLAGS=0x");
  if (irq < 16) {
    Serial1.print('0');
  }
  Serial1.print(irq, HEX);
  Serial1.print(" TX_DONE_in_IRQ=");
  Serial1.print((irq & RH_RF95_TX_DONE) ? 1 : 0);
  Serial1.print(" DIO0_pin=");
  Serial1.print(dio0);
  if (ver == 0x00 || ver == 0xff) {
    Serial1.print(" (VERSION 0x00/0xFF often means MISO/CS/SPI mode wiring)");
  }
  Serial1.println();
}

// ========== LORA SEND ==========
bool sendLoRa(uint8_t *data, uint8_t len) {
  if (!loraReady) {
    Serial1.println("[LoRa] TX skipped (radio not initialized)");
    return false;
  }

  // rf95.send() starts with waitPacketSent() for any in-flight TX — bound it so a
  // lost DIO0 IRQ cannot deadlock before the new packet is queued.
  interrupts();
  if (!rf95.waitPacketSent(LORA_WAIT_PACKET_SENT_TIMEOUT_MS)) {
    // RISING on DIO0 can miss TX-done; chip may still have TX_DONE in IRQ_FLAGS.
    uint8_t irqPre = loraSpiReadReg8(RH_RF95_REG_12_IRQ_FLAGS);
    if (irqPre & RH_RF95_TX_DONE) {
      rf95.pollRadioInterrupts();
    }
    if (!rf95.waitPacketSent(2)) {
      Serial1.print("[LoRa] warning: radio still in TX after ");
      Serial1.print(LORA_WAIT_PACKET_SENT_TIMEOUT_MS);
      Serial1.println(
          " ms — snapshot then forcing idle (GPIO IRQ path suspect)");
      logLoRaRadioSnapshot("stuck TX before send()");
      rf95.setModeIdle();
    }
  }

  Serial1.print("[LoRa] rf95.send(), len=");
  Serial1.print(len);
  Serial1.println(" ...");

  if (!rf95.send(data, len)) {
    Serial1.println("[LoRa] rf95.send() returned false");
    return false;
  }

  // RadioHead clears RHModeTx from DIO0 ISR only; RISING can miss. Poll IRQ_FLAGS
  // within the same slot-duration budget so we do not wait the full timeout when
  // TX_DONE is already set (common when the GPIO edge is missed).
  interrupts();
  const uint32_t txWaitT0 = millis();
  bool txDoneSynced = false;
  while ((millis() - txWaitT0) < LORA_WAIT_PACKET_SENT_TIMEOUT_MS) {
    uint8_t irq = loraSpiReadReg8(RH_RF95_REG_12_IRQ_FLAGS);
    if (irq & RH_RF95_TX_DONE) {
      rf95.pollRadioInterrupts();
      if (rf95.waitPacketSent(2)) {
        txDoneSynced = true;
        break;
      }
    }
    if (rf95.waitPacketSent(1)) {
      txDoneSynced = true;
      break;
    }
    yield();
  }
  if (!txDoneSynced) {
    uint8_t irqPoll = loraSpiReadReg8(RH_RF95_REG_12_IRQ_FLAGS);
    if (irqPoll & RH_RF95_TX_DONE) {
      rf95.pollRadioInterrupts();
      txDoneSynced = rf95.waitPacketSent(2);
    }
  }
  if (!txDoneSynced) {
    Serial1.print("[LoRa] ERROR: TX wait failed after ");
    Serial1.print(LORA_WAIT_PACKET_SENT_TIMEOUT_MS);
    Serial1.println(" ms (slot duration) — snapshot then idle.");
    logLoRaRadioSnapshot("TX-done wait timeout");
    rf95.setModeIdle();
    return false;
  }
  return true;
}

// ========== TRANSMISSION LOGIC ==========
void processScheduledTransmission() {
  static uint32_t lastWaitingLogMs = 0;
  static bool logged_this_tx_window = false;

  if (!txWindowOpen || !txWindowSendArmed || !packet_ready) {
    if (!txWindowOpen || !txWindowSendArmed) {
      logged_this_tx_window = false;
    }
    uint32_t now = millis();
    if (txWindowOpen && txWindowSendArmed && !packet_ready &&
        now - lastWaitingLogMs >= 3000) {
      lastWaitingLogMs = now;
      Serial1.println("[LoRa] TX window open, armed, but packet_ready=false (need I2C telemetry)");
    }
    return;
  }

  if (!logged_this_tx_window) {
    Serial1.println("[LoRa] TX window: copying snapshot and transmitting...");
    logged_this_tx_window = true;
  }

  LoRaTransmitPacket tx_copy;

  noInterrupts();
  memcpy(tx_copy.callsign, (const void *)lora_tx_buffer.callsign,
         LORA_CALLSIGN_SIZE);
  tx_copy.stage_id[0] = lora_tx_buffer.stage_id[0];
  memcpy(&tx_copy.telemetry, &telemetryRing[telemetryRingNewestIdx].gps,
         sizeof(LoRaTelemetryPayload));
  interrupts();

  if (sendLoRa(reinterpret_cast<uint8_t *>(&tx_copy), sizeof(tx_copy))) {
    Serial1.println("[LoRa] TX completed successfully");
    ledPulseLoRaTransmit();
    logged_this_tx_window = false;
    noInterrupts();
    packet_ready = false;
    txWindowSendArmed = false;
    interrupts();
  } else {
    Serial1.println("[LoRa] TX failed (will retry while window allows)");
  }
}

// ========== SETUP ==========
void setup() {
  Serial1.begin(115200);
  delay(1000);

  enableCycleCounter();

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), pps_isr, RISING);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  SPI.begin();

  if (!rf95.init()) {
    Serial1.println("LoRa radio init failed");
  } else if (!rf95.setFrequency(RF95_FREQ)) {
    Serial1.println("LoRa frequency set failed");
  } else {
    rf95.setTxPower(23, false);
    loraReady = true;
    Serial1.println("LoRa radio initialized");
    logLoRaRadioSnapshot("after init (expect VERSION=0x12 for SX1276/78)");
  }

  Wire1.begin(I2C_RECEIVE_ADDRESS);
  Wire1.onReceive(receiveI2C);
  memcpy((void *)lora_tx_buffer.callsign, CALLSIGN, LORA_CALLSIGN_SIZE);
  lora_tx_buffer.stage_id[0] = (uint8_t)SLOT_TYPE;

#if JACOB_I2C_TELEMETRY_VERBOSE
  Serial1.println("[I2C dbg] Telemetry wire layout (must match Spencer/SD):");
  Serial1.print("[I2C dbg] sizeof(TelemetryData)=");
  Serial1.print(sizeof(TelemetryData));
  Serial1.print(" TELEMETRY_PACKET_MAX_BYTES=");
  Serial1.print(TELEMETRY_PACKET_MAX_BYTES);
  Serial1.print(" offsetof(gps.unix)=");
  Serial1.print(
      (unsigned)(offsetof(TelemetryData, gps) + offsetof(GPSData, unixEpoch)));
  Serial1.print(" offsetof(bmp)=");
  Serial1.println((unsigned)offsetof(TelemetryData, bmp));
#endif

  Serial1.println("System Initialized");
  Serial1.println("LED: double short flash = I2C telemetry packet complete; long flash = LoRa TX done");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

// ========== LOOP ==========
void loop() {
  getCycles64();     // maintain counter
  // Multiple passes drain frames that arrived while processing (verbose Serial
  // used to slow this enough by accident); keep PPS/unix scheduling correct with
  // verbose off.
  for (uint8_t drainPass = 0; drainPass < 6; drainPass++) {
    processQueuedI2CFrames();
  }

  if (pps_flag) {
    noInterrupts();
    uint64_t now = current_pps_cycles;
    pps_flag = false;
    interrupts();

    handlePPS(now);
  }

  processScheduledTransmission();
}
