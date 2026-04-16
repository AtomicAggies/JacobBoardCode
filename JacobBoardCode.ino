#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>

// ================= CONFIG =================
#define PPS_PIN   8
#define CPU_HZ    600000000ULL

#define TX_CYCLE_MS 1400

#define SLOT_SUSTAINER_START 450
#define SLOT_DURATION        350

#define RF95_FREQ 434.0

#define RFM95_CS 10
#define RFM95_RST 2
#define RFM95_INT 3
#define epoch 1400 //epoch in ms

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, 2);

// Adjustable send window (DEFAULT = 40 ms)
uint32_t TX_READY_WINDOW_MS = 40;

// SPI
#define LORA_CS 10

// I2C
#define I2C_ADDRESS 0x08

// Ham radio callsign and LoRa frequency
#define CALLSIGN        "KI5VVL"
#define LORA_FREQUENCY  434

// =============== PACKET CONFIG ===============
#define CALLSIGN_SIZE 6
#define ALTITUDE_SIZE 24
#define MAG_SIZE 16
#define ACCEL_SIZE 96
#define GPS_SIZE 92
#define TEMP_SIZE 16
#define STAGE_ID 1

const uint8_t PACKET_SIZE = CALLSIGN_SIZE + 1 + ALTITUDE_SIZE + MAG_SIZE + ACCEL_SIZE + GPS_SIZE + TEMP_SIZE;

struct TelemetryPacket {
  uint8_t callsign[CALLSIGN_SIZE];
  uint8_t stage_id;
  uint8_t altitude[ALTITUDE_SIZE];
  uint8_t magnetometer[MAG_SIZE];
  uint8_t accelerometer[ACCEL_SIZE];
  uint8_t gps[GPS_SIZE];
  uint8_t temperature[TEMP_SIZE];
};

volatile TelemetryPacket rx_buffer;
volatile bool packet_ready = false;

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

// ========== RECEIVE I2C PACKET ==========
void receiveI2C(int bytes) {
  static uint8_t temp[PACKET_SIZE];
  static uint16_t idx = 0;

  while (Wire.available()) {
    uint8_t byte = Wire.read();
    Serial.print((char)byte);

    temp[idx++] = byte;

    if (idx >= PACKET_SIZE - CALLSIGN_SIZE - 1) {
      memcpy((void*)&rx_buffer, CALLSIGN, CALLSIGN_SIZE);
      memset((void*)&rx_buffer+CALLSIGN_SIZE, STAGE_ID, 1);
      memcpy((void*)&rx_buffer+CALLSIGN_SIZE+1, temp, PACKET_SIZE);
      packet_ready = true;
      idx = 0;

      Serial.println("Packet received");
      TelemetryPacket tx_copy;

      noInterrupts();
      memcpy(&tx_copy, (const void*)&rx_buffer, sizeof(rx_buffer));
      interrupts();

      rf95.send((uint8_t*)&tx_copy, sizeof(tx_copy));
      rf95.waitPacketSent();
    }
  }
}

// ========== SPI SEND ==========
void sendSPI(uint8_t* data, uint16_t len) {
  digitalWrite(LORA_CS, LOW);

  for (uint16_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }

  digitalWrite(LORA_CS, HIGH);

  Serial.println("Packet sent via SPI");
}

// ========== TRANSMISSION LOGIC ==========
bool already_sent = false;

void handleTransmission() {
  uint32_t t = getCycleTimeMs();

  if (t >= SLOT_SUSTAINER_START &&
      t < (SLOT_SUSTAINER_START + SLOT_DURATION)) {

    if (!already_sent &&
        t <= (SLOT_SUSTAINER_START + TX_READY_WINDOW_MS)) {

      if (packet_ready) {
        sendSPI((uint8_t*)&rx_buffer, PACKET_SIZE);
        packet_ready = false;
        already_sent = true;
      }
    }

  } else {
    already_sent = false;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  enableCycleCounter();

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), pps_isr, RISING);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  SPI.begin();

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveI2C);

  Serial.println("System Initialized");
}

// ========== LOOP ==========
void loop() {
  getCycles64();     // maintain counter
  // receivePacket();   // fill buffer

  if (pps_flag) {
    noInterrupts();
    uint64_t now = current_pps_cycles;
    pps_flag = false;
    interrupts();

    handlePPS(now);
  }

  handleTransmission();

  // Debug timing output (optional)
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();

    Serial.print("UTC: ");
    Serial.print(utc_seconds);
    Serial.print(" | Cycle ms: ");
    Serial.print(getCycleTimeMs());
    Serial.print(" | Window(ms): ");
    Serial.println(TX_READY_WINDOW_MS);
  }
}