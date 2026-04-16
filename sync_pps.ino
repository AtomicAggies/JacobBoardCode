// ===== CONFIG =====
#define PPS_PIN   8
#define CPU_HZ    600000000

// ===== 64-bit CYCLE COUNTER STATE =====
volatile uint32_t last_cycle_low = 0;
volatile uint64_t cycle_high = 0;

// Extend DWT_CYCCNT to 64 bits
uint64_t getCycles64() {
  uint32_t low = ARM_DWT_CYCCNT;

  if (low < last_cycle_low) {
    // Detect rollover
    cycle_high += (1ULL << 32);
  }

  last_cycle_low = low;
  return (cycle_high | low);
}

// ===== PPS STATE =====
volatile uint64_t last_pps_cycles = 0;
volatile uint64_t current_pps_cycles = 0;
volatile bool pps_flag = false;

// ===== CLOCK DISCIPLINE =====
double freq_correction = 0.0;
double phase_correction = 0.0;

// Tunable gains
const double Kf = 1e-12;
const double Kp = 1e-3;

// ===== UTC TIME =====
uint64_t utc_seconds = 0;

// ===== ENABLE CYCLE COUNTER =====
void enableCycleCounter() {
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

// ===== PPS ISR =====
void pps_isr() {
  current_pps_cycles = getCycles64();
  pps_flag = true;
}

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  delay(1000);

  enableCycleCounter();

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), pps_isr, RISING);

  Serial.println("64-bit PPS Sync Start");
}

// ===== LOOP =====
void loop() {
  // Keep cycle counter extended (IMPORTANT: call frequently)
  getCycles64();

  if (pps_flag) {
    noInterrupts();
    uint64_t now = current_pps_cycles;
    pps_flag = false;
    interrupts();

    if (last_pps_cycles != 0) {
      uint64_t delta = now - last_pps_cycles;

      double expected = (double)CPU_HZ;
      double error = (double)delta - expected;

      // Frequency correction (slow)
      freq_correction += Kf * error;

      // Phase correction (faster)
      phase_correction += Kp * error;

      Serial.print("delta: ");
      Serial.print((uint32_t)delta);
      Serial.print(" error: ");
      Serial.print(error);
      Serial.print(" freq_corr: ");
      Serial.println(freq_correction, 12);
    }

    last_pps_cycles = now;

    // Increment UTC second (replace with NMEA sync later)
    utc_seconds++;
  }

  // ===== CURRENT TIME COMPUTATION =====
  uint64_t now_cycles = getCycles64();

  uint64_t pps_cycles_snapshot;
  noInterrupts();
  pps_cycles_snapshot = last_pps_cycles;
  interrupts();

  uint64_t delta_cycles = now_cycles - pps_cycles_snapshot;

  // Apply corrections
  double corrected_cycles =
      (double)delta_cycles * (1.0 + freq_correction) + phase_correction;

  double seconds_since_pps = corrected_cycles / CPU_HZ;

  // ===== OUTPUT =====
  Serial.print("UTC: ");
  Serial.print(utc_seconds);
  Serial.print(" + ");
  Serial.print(seconds_since_pps, 9);
  Serial.println(" s");

  delay(200);
}