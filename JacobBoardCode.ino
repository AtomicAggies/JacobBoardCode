#include <Arduino.h>
#include <IntervalTimer.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <stddef.h>

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
// 2 = payload
#define SLOT_TYPE 1

#define RF95_FREQ 434.0

#define RFM95_CS 10
#define RFM95_RST 2
#define RFM95_INT 3
// Epoch length in milliseconds.
// Recommended range: 1000..10000 ms.
// Choose it from slot planning math:
//   slot_spacing_ms = EPOCH
//   must satisfy slot_spacing_ms >= LoRa airtime_ms + guard_ms
// Example: 180 ms airtime + 10 ms guard => EPOCH >= 190 ms
// (then pick a larger practical value like 1400 ms for sparse traffic).
#define EPOCH 1400

#define CALLSIGN "KJ5NPP"

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Adjustable send window (DEFAULT = 40 ms)
uint32_t TX_READY_WINDOW_MS = 40;

// Define the start time of the slot for this board
uint32_t SLOT_START =
  SLOT_TYPE == 0 ? SLOT_BOOSTER_START :
  SLOT_TYPE == 1 ? SLOT_SUSTAINER_START :
  SLOT_TYPE == 2 ? SLOT_PAYLOAD_START : 0;

// I2C framing. This matches SpencerBoardCode's telemetry sender and
// AbrahamBoardCode's receiver: Spencer sends one packed TelemetryData as
// multiple 32-byte I2C frames addressed to this board's I2C slave address.
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
const uint8_t I2C_FRAME_DESTINATION_RADIO = 1 << 1;
const uint8_t I2C_FRAME_START = 1 << 7;
const uint8_t FRAME_QUEUE_SIZE = 8;
const unsigned long PACKET_RECEIVE_TIMEOUT_MS = 1000;

struct I2CFrame {
  uint8_t length;
  uint8_t bytes[I2C_FRAME_MAX_SIZE];
};

// Matches SpencerBoardCode.ino (sensor TX). I2C carries the full TelemetryData;
// LoRa sends only GPS, BMP, magnetometer, and inertial (no counter, validity, or
// lastI2C* bookkeeping).
struct __attribute__((packed)) GPSData {
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  int32_t nedNorthVel;
  int32_t nedDownVel;
  int32_t nedEastVel;
  uint32_t unixEpoch;
};

struct __attribute__((packed)) BMPData {
  float temperature;
  float pressure;
};

struct __attribute__((packed)) MagnetometerData {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct __attribute__((packed)) InertialData {
  float temperature;
  float gyroX;
  float gyroY;
  float gyroZ;
  float accelX;
  float accelY;
  float accelZ;
};

struct __attribute__((packed)) TelemetryData {
  uint16_t packetCounter;
  uint8_t validity;
  GPSData gps;
  BMPData bmp;
  MagnetometerData magnetometer;
  InertialData inertial;
  uint8_t lastI2CBytesWritten;
  uint8_t lastI2CStatus;
};

static_assert(sizeof(TelemetryData) == 75,
              "TelemetryData must be 75 bytes; update I2C/LoRa peers if layout changes");
const uint8_t TELEMETRY_PACKET_SIZE = sizeof(TelemetryData);

// Radio payload: four sensor blocks only (matches TelemetryData from gps onward).
struct __attribute__((packed)) LoRaTelemetryPayload {
  GPSData gps;
  BMPData bmp;
  MagnetometerData magnetometer;
  InertialData inertial;
};

static_assert(sizeof(LoRaTelemetryPayload) ==
                  offsetof(TelemetryData, lastI2CBytesWritten) -
                      offsetof(TelemetryData, gps),
              "LoRaTelemetryPayload must match TelemetryData gps..inclusive");

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

    Serial.print("PPS delta: ");
    Serial.print((uint32_t)delta);
    Serial.print(" error: ");
    Serial.println(error);
  }

  last_pps_cycles = now;
  utc_seconds++;

  txWindowOpen = false;
  txWindowSendArmed = false;
  txWindowOpenTimer.end();
  txWindowCloseTimer.end();

  if (!hasTelemetryUnixEpochSeconds || EPOCH == 0) {
    return;
  }

  uint64_t unixMsAtPps = (uint64_t)latestTelemetryUnixEpochSeconds * 1000ULL;
  uint32_t epochPhaseMs = (uint32_t)(unixMsAtPps % EPOCH);
  uint32_t slotPhaseMs = (uint32_t)(SLOT_START % EPOCH);
  uint32_t timeToSlotMs = 0;
  if (epochPhaseMs <= slotPhaseMs) {
    timeToSlotMs = slotPhaseMs - epochPhaseMs;
  } else {
    timeToSlotMs = EPOCH - (epochPhaseMs - slotPhaseMs);
  }

  if (timeToSlotMs >= 1000) {
    return;
  }

  txWindowOpenTimer.begin(onTxWindowOpen, timeToSlotMs * 1000);
  uint32_t closeDelayMs = timeToSlotMs + TX_READY_WINDOW_MS;
  if (closeDelayMs > 999) {
    closeDelayMs = 999;
  }
  txWindowCloseTimer.begin(onTxWindowClose, closeDelayMs * 1000);
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

// ========== I2C PACKET ASSEMBLY ==========
uint8_t checksumI2CPayload(const uint8_t *payload, uint8_t payloadSize) {
  uint8_t checksum = 0;
  for (uint8_t index = 0; index < payloadSize; index++) {
    checksum += payload[index];
  }
  return checksum;
}

void discardPartialPacket(const char *reason) {
  if (receivingPacket || i2cTelemetryBytesReceived > 0) {
    invalidPacketCount++;
    Serial.print("Discarded partial telemetry packet (");
    Serial.print(i2cTelemetryBytesReceived);
    Serial.print("/");
    Serial.print(TELEMETRY_PACKET_SIZE);
    Serial.print(" bytes): ");
    Serial.println(reason);
  }

  i2cTelemetryBytesReceived = 0;
  receivingPacket = false;
  lastPacketFrameMillis = 0;
}

void markTelemetryPacketReady() {
  noInterrupts();
  memcpy((void *)&lora_tx_buffer.telemetry, &i2cTelemetryBuffer.gps,
         sizeof(LoRaTelemetryPayload));
  packet_ready = true;
  interrupts();
  latestTelemetryUnixEpochSeconds = i2cTelemetryBuffer.gps.unixEpoch;
  hasTelemetryUnixEpochSeconds = latestTelemetryUnixEpochSeconds != 0;

  validPacketCount++;
  Serial.print("Telemetry packet ready for LoRa TX: ");
  Serial.println(validPacketCount);
}

void processI2CFrame(const I2CFrame &frame) {
  if (frame.length < I2C_FRAME_HEADER_SIZE) {
    Serial.print("Discarded invalid I2C frame shorter than header: ");
    Serial.println(frame.length);
    discardPartialPacket("short I2C frame");
    return;
  }

  uint8_t frameFlags = frame.bytes[0];
  uint8_t receivedChecksum = frame.bytes[1];
  uint8_t payloadSize = frame.length - I2C_FRAME_HEADER_SIZE;
  const uint8_t *payload = frame.bytes + I2C_FRAME_HEADER_SIZE;
  bool isStartFrame = (frameFlags & I2C_FRAME_START) != 0;

  if ((frameFlags & I2C_FRAME_DESTINATION_RADIO) == 0) {
    ignoredFrameCount++;
    return;
  }

  uint8_t calculatedChecksum = checksumI2CPayload(payload, payloadSize);
  if (calculatedChecksum != receivedChecksum) {
    checksumFailureCount++;
    Serial.print("Checksum failure on I2C frame (received 0x");
    Serial.print(receivedChecksum, HEX);
    Serial.print(", calculated 0x");
    Serial.print(calculatedChecksum, HEX);
    Serial.println(")");
    discardPartialPacket("I2C frame checksum failure");
    return;
  }

  if (isStartFrame) {
    discardPartialPacket("new telemetry packet started before previous packet was complete");
    receivingPacket = true;
    lastPacketFrameMillis = millis();
  } else if (!receivingPacket) {
    Serial.println("Discarded continuation I2C frame with no active telemetry packet");
    return;
  }

  if (payloadSize == 0) {
    discardPartialPacket("empty I2C frame payload");
    return;
  }

  if (i2cTelemetryBytesReceived + payloadSize > TELEMETRY_PACKET_SIZE) {
    discardPartialPacket("I2C frame would overflow telemetry buffer");
    return;
  }

  memcpy(i2cTelemetryBytes() + i2cTelemetryBytesReceived, payload, payloadSize);
  i2cTelemetryBytesReceived += payloadSize;
  lastPacketFrameMillis = millis();

  if (i2cTelemetryBytesReceived == TELEMETRY_PACKET_SIZE) {
    markTelemetryPacketReady();
    i2cTelemetryBytesReceived = 0;
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

// ========== RECEIVE I2C FRAME ISR ==========
void receiveI2C(int count) {
  if (count > I2C_FRAME_MAX_SIZE) {
    while (Wire.available()) {
      Wire.read();
    }
    droppedFrameCount++;
    return;
  }

  uint8_t nextHead = (frameQueueHead + 1) % FRAME_QUEUE_SIZE;
  if (nextHead == frameQueueTail) {
    while (Wire.available()) {
      Wire.read();
    }
    droppedFrameCount++;
    return;
  }

  I2CFrame &frame = frameQueue[frameQueueHead];
  frame.length = 0;
  bool isForRadio = false;
  while (Wire.available()) {
    uint8_t byteValue = Wire.read();
    if (frame.length < I2C_FRAME_MAX_SIZE) {
      frame.bytes[frame.length++] = byteValue;
      if (frame.length == 1) {
        isForRadio = (byteValue & I2C_FRAME_DESTINATION_RADIO) != 0;
      }
    } else {
      droppedFrameCount++;
    }
  }

  if (frame.length < I2C_FRAME_HEADER_SIZE) {
    return;
  }

  if (!isForRadio) {
    ignoredFrameCount++;
    return;
  }

  frameQueueHead = nextHead;
}

void processQueuedI2CFrames() {
  static uint16_t lastDroppedFrameCount = 0;

  noInterrupts();
  uint16_t currentDroppedFrameCount = droppedFrameCount;
  interrupts();

  if (currentDroppedFrameCount != lastDroppedFrameCount) {
    uint16_t droppedSinceLastLog = currentDroppedFrameCount - lastDroppedFrameCount;
    lastDroppedFrameCount = currentDroppedFrameCount;
    Serial.print("WARNING: I2C frame queue overflow dropped ");
    Serial.print(droppedSinceLastLog);
    Serial.println(" frame(s)");
    discardPartialPacket("I2C frame queue overflow");
  }

  I2CFrame frame;
  while (popQueuedFrame(frame)) {
    processI2CFrame(frame);
  }

  if (receivingPacket && lastPacketFrameMillis != 0 &&
      millis() - lastPacketFrameMillis > PACKET_RECEIVE_TIMEOUT_MS) {
    discardPartialPacket("timed out before full telemetry packet received");
  }
}

// ========== LORA SEND ==========
bool sendLoRa(uint8_t* data, uint8_t len) {
  if (!loraReady) {
    Serial.println("LoRa TX skipped because radio is not initialized");
    return false;
  }

  if (!rf95.send(data, len)) {
    Serial.println("LoRa TX failed to start");
    return false;
  }

  rf95.waitPacketSent();
  Serial.println("Packet sent via LoRa");
  return true;
}

// ========== TRANSMISSION LOGIC ==========
void processScheduledTransmission() {
  if (!txWindowOpen || !txWindowSendArmed || !packet_ready) {
    return;
  }

  LoRaTransmitPacket tx_copy;

  noInterrupts();
  memcpy(&tx_copy, (const void*)&lora_tx_buffer, sizeof(tx_copy));
  interrupts();

  if (sendLoRa(reinterpret_cast<uint8_t*>(&tx_copy), sizeof(tx_copy))) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    noInterrupts();
    packet_ready = false;
    txWindowSendArmed = false;
    interrupts();
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  enableCycleCounter();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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
    Serial.println("LoRa radio init failed");
  } else if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("LoRa frequency set failed");
  } else {
    rf95.setTxPower(23, false);
    loraReady = true;
    Serial.println("LoRa radio initialized");
  }

  Wire.begin(I2C_RECEIVE_ADDRESS);
  Wire.onReceive(receiveI2C);
  memcpy((void *)lora_tx_buffer.callsign, CALLSIGN, LORA_CALLSIGN_SIZE);
  lora_tx_buffer.stage_id[0] = (uint8_t)SLOT_TYPE;

  Serial.println("System Initialized");
}

// ========== LOOP ==========
void loop() {
  getCycles64();     // maintain counter
  processQueuedI2CFrames();

  if (pps_flag) {
    noInterrupts();
    uint64_t now = current_pps_cycles;
    pps_flag = false;
    interrupts();

    handlePPS(now);
  }

  processScheduledTransmission();

  // Debug timing output (optional)
  // static uint32_t lastPrint = 0;
  // if (millis() - lastPrint > 500) {
  //   lastPrint = millis();

  //   Serial.print("UTC: ");
  //   Serial.print(utc_seconds);
  //   Serial.print(" | Cycle ms: ");
  //   Serial.print(getCycleTimeMs());
  //   Serial.print(" | Window(ms): ");
  //   Serial.print(TX_READY_WINDOW_MS);
  //   Serial.print(" | Ready: ");
  //   Serial.print(packet_ready ? "yes" : "no");
  //   Serial.print(" | Valid: ");
  //   Serial.print(validPacketCount);
  //   Serial.print(" | Invalid: ");
  //   Serial.print(invalidPacketCount);
  //   Serial.print(" | Ignored: ");
  //   Serial.print(ignoredFrameCount);
  //   Serial.print(" | Checksum failures: ");
  //   Serial.println(checksumFailureCount);
  // }
}
