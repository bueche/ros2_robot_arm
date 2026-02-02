#include <Wire.h>
#include <Adafruit_INA219.h>
#include <INA226.h>  // Library for INA226

// ---------- I2C Bus 0 pins (INA219 sensors) ----------
#define I2C0_SDA 1
#define I2C0_SCL 2

// ---------- I2C Bus 1 pins (INA226 sensor) ----------
// ESP32-S3 - INA226 is connected to these pins
// Exact brand: ESP32-S3 N16R8 (Lonely Binary) found on Amazon
#define I2C1_SDA 21
#define I2C1_SCL 47

// Create second I2C bus object
TwoWire I2C_Bus1 = TwoWire(1);

// ---------- INA219 sensors on Bus 0 ----------
// 12V rail on address 0x40 (default)
Adafruit_INA219 ina219_12v(0x40);
// 5V rail on address 0x41 (second module, address jumper set!)
Adafruit_INA219 ina219_5v(0x41);

// ---------- INA226 sensor on Bus 1 ----------
// Can use address 0x44 or 0x40(found by I2C scanner.. but varies per INA226 sensor)
// The MECCANIXITY INA226 (sold on Amazon) sensors are preset with different addresses
// when purchased as a group of sensors. You can't assume a default.
INA226 ina226_5v(0x44, &I2C_Bus1);

// ---------- Streaming control ----------
static bool     streaming        = true;   // start streaming by default
static uint32_t sample_period_ms = 100;    // 100 ms = 10 Hz

// ---------- Statistics for sensor comparison ----------
struct SensorStats {
  float sum;
  float sum_sq;
  int count;
  float min_val;
  float max_val;
  
  void reset() {
    sum = 0;
    sum_sq = 0;
    count = 0;
    min_val = 999999;
    max_val = -999999;
  }
  
  void add(float val) {
    sum += val;
    sum_sq += val * val;
    count++;
    if (val < min_val) min_val = val;
    if (val > max_val) max_val = val;
  }
  
  float mean() { return count > 0 ? sum / count : 0; }
  float stddev() { 
    if (count < 2) return 0;
    float mean_val = mean();
    return sqrt(sum_sq / count - mean_val * mean_val);
  }
};

SensorStats stats_ina219_5v;
SensorStats stats_ina226_5v;
SensorStats stats_diff;

// ---------- Helpers ----------
void printStats() {
  Serial.println("\n# === 5V RAIL SENSOR COMPARISON STATISTICS ===");
  
  Serial.print("# INA219 (5V): mean=");
  Serial.print(stats_ina219_5v.mean() * 1000, 2);
  Serial.print("mA, std=");
  Serial.print(stats_ina219_5v.stddev() * 1000, 2);
  Serial.print("mA, min=");
  Serial.print(stats_ina219_5v.min_val * 1000, 2);
  Serial.print("mA, max=");
  Serial.print(stats_ina219_5v.max_val * 1000, 2);
  Serial.println("mA");
  
  Serial.print("# INA226 (5V): mean=");
  Serial.print(stats_ina226_5v.mean() * 1000, 2);
  Serial.print("mA, std=");
  Serial.print(stats_ina226_5v.stddev() * 1000, 2);
  Serial.print("mA, min=");
  Serial.print(stats_ina226_5v.min_val * 1000, 2);
  Serial.print("mA, max=");
  Serial.print(stats_ina226_5v.max_val * 1000, 2);
  Serial.println("mA");
  
  Serial.print("# Difference (INA219-INA226): mean=");
  Serial.print(stats_diff.mean() * 1000, 2);
  Serial.print("mA, std=");
  Serial.print(stats_diff.stddev() * 1000, 2);
  Serial.print("mA, min=");
  Serial.print(stats_diff.min_val * 1000, 2);
  Serial.print("mA, max=");
  Serial.print(stats_diff.max_val * 1000, 2);
  Serial.println("mA");
  
  float mean_diff = stats_diff.mean() * 1000;
  float mean_avg = (stats_ina219_5v.mean() + stats_ina226_5v.mean()) / 2.0 * 1000;
  if (mean_avg > 10) {  // Only show % if current is significant
    float percent_diff = (mean_diff / mean_avg) * 100;
    Serial.print("# Percent difference: ");
    Serial.print(percent_diff, 2);
    Serial.println("%");
  }
  
  Serial.print("# Sample count: ");
  Serial.println(stats_ina219_5v.count);
  Serial.println("# ");
}

uint16_t readCalibrationRegister() {
  Wire1.beginTransmission(0x44);
  Wire1.write(0x05);  // Calibration register
  Wire1.endTransmission(false);
  
  Wire1.requestFrom(0x44, (uint8_t)2);
  if (Wire1.available() == 2) {
    uint8_t msb = Wire1.read();
    uint8_t lsb = Wire1.read();
    return (msb << 8) | lsb;
  }
  return 0xFFFF;
}

void handleSerialCommands() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    line.toUpperCase();

    if (line.startsWith("S")) {
      int val = 0;
      if (line.length() >= 2) {
        val = line.substring(1).toInt();
      }
      streaming = (val != 0);
      Serial.print("# streaming=");
      Serial.println(streaming ? "1" : "0");
      
      // Reset stats when toggling streaming
      if (streaming) {
        stats_ina219_5v.reset();
        stats_ina226_5v.reset();
        stats_diff.reset();
      }

    } else if (line.startsWith("P")) {
      int v = line.substring(1).toInt();
      if (v >= 10 && v <= 5000) {
        sample_period_ms = (uint32_t)v;
        Serial.print("# period_ms=");
        Serial.println(sample_period_ms);
      } else {
        Serial.println("# invalid period (10..5000 ms)");
      }

    } else if (line == "T") {
      // Print statistics
      printStats();
      
    } else if (line == "R") {
      // Reset statistics
      stats_ina219_5v.reset();
      stats_ina226_5v.reset();
      stats_diff.reset();
      Serial.println("# Statistics reset");

    } else if (line == "H") {
      Serial.println("# Commands:");
      Serial.println("#   S 1   -> start streaming");
      Serial.println("#   S 0   -> stop streaming");
      Serial.println("#   pNNN  -> set period in ms (e.g. p50, p100)");
      Serial.println("#   T     -> print 5V sensor comparison statistics");
      Serial.println("#   R     -> reset statistics");
      Serial.println("#   H     -> this help");
    } else {
      Serial.print("# unknown cmd: ");
      Serial.println(line);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("# Three-Sensor Monitor: DUAL I2C BUS Configuration (ESP32-S3)");
  Serial.println("# I2C Bus 0 (GPIO1/2): INA219 (12V) + INA219 (5V)");
  Serial.println("# I2C Bus 1 (GPIO21/47): INA226 (5V) in series with INA219 (5V)");
  Serial.println("# CSV columns:");
  Serial.println("# t_us,"
                 "bus12_v,shunt12_mv,current12_A,power12_W,"
                 "bus5v_ina219_v,shunt5v_ina219_mv,current5v_ina219_A,power5v_ina219_W,"
                 "bus5v_ina226_v,shunt5v_ina226_mv,current5v_ina226_A,power5v_ina226_W,"
                 "current5v_diff_mA,current5v_avg_mA");
  Serial.println("# Commands: 'S 1', 'S 0', 'pNNN', 'T' (stats), 'R' (reset), 'H'");

  // ========== Initialize I2C Bus 0 (INA219 sensors) ==========
  Serial.println("# Initializing I2C Bus 0 (GPIO1=SDA, GPIO2=SCL)...");
  Wire.begin(I2C0_SDA, I2C0_SCL);

  // ========== Initialize I2C Bus 1 (INA226 sensor) ==========
  Serial.println("# Initializing I2C Bus 1 (GPIO21=SDA, GPIO47=SCL)...");
  I2C_Bus1.begin(I2C1_SDA, I2C1_SCL);

  // ========== Initialize 12V rail sensor on Bus 0 ==========
  Serial.println("# Initializing INA219 (12V) at address 0x40 on Bus 0...");
  if (!ina219_12v.begin(&Wire)) {
    Serial.println("# WARNING: Failed to find INA219 (12V) at 0x40");
    Serial.println("# 12V rail will show zero values");
    Serial.println("# Continuing with 5V rail sensors...");
  } else {
    // Use 32V_1A calibration for better resolution
    ina219_12v.setCalibration_32V_1A();
    Serial.println("# ✓ INA219 (12V) initialized at 0x40 on Bus 0");
    Serial.println("#   Calibration: 32V_1A");
  }

  // ========== Initialize 5V rail INA219 on Bus 0 ==========
  Serial.println("# Initializing INA219 (5V) at address 0x41 on Bus 0...");
  if (!ina219_5v.begin(&Wire)) {
    Serial.println("# ERROR: Failed to find INA219 (5V) at 0x41");
    Serial.println("# Check wiring and I2C address!");
    while (1) {
      delay(500);
      Serial.println("# still failing ina219_5v.begin()");
    }
  }

  // Use 32V_1A calibration for better resolution in 0-1.3A range
  ina219_5v.setCalibration_32V_1A();
  Serial.println("# ✓ INA219 (5V) initialized at 0x41 on Bus 0");
  Serial.println("#   Calibration: 32V_1A");
  Serial.println("#   Resolution: 0.04mA (40µA)");
  Serial.println("#   Max current: ~1.3A accurate");

  // ========== Initialize 5V rail INA226 on Bus 1 ==========
  Serial.println("# Initializing INA226 (5V) at address 0x44 on Bus 1...");
  if (!ina226_5v.begin()) {
    Serial.println("# ERROR: Failed to find INA226 at 0x44 on Bus 1");
    Serial.println("# Check wiring to GPIO21/GPIO47!");
    Serial.println("# INA226 wiring:");
    Serial.println("#   VIN+/VIN- in series with INA219");
    Serial.println("#   SDA to GPIO21 (right side)");
    Serial.println("#   SCL to GPIO47 (right side, near bottom)");
    Serial.println("#   VCC to 3.3V or 5V");
    Serial.println("#   GND to GND");
    while (1) {
      delay(500);
      Serial.println("# still failing ina226_5v.begin()");
    }
  }
  
  // Rob Tillaart library requires explicit calibration!
  // Set shunt resistor value and max expected current
  int result = ina226_5v.setMaxCurrentShunt(0.800, 0.1);  // 0.819A max, 0.1 ohm shunt
  Serial.print("setMaxCurrentShunt() returned: ");
  Serial.println(result);
  // Check for error codes
  if (result == INA226_ERR_NONE) {
    Serial.println("No errors");
  } else if (result == INA226_ERR_SHUNTVOLTAGE_HIGH) {
    Serial.println("ERROR: Shunt voltage too high!");
  } else if (result == INA226_ERR_MAXCURRENT_LOW) {
    Serial.println("ERROR: Max current too low!");
  } else if (result == INA226_ERR_SHUNT_LOW) {
    Serial.println("ERROR: Shunt resistance too low!");
  } else if (result == INA226_ERR_NORMALIZE_FAILED) {
    Serial.println("ERROR: Normalization failed!");
  }
  // NOW READ THE CALIBRATION REGISTER TO SEE IF IT WAS SET
  if (!ina226_5v.isCalibrated()) {
    Serial.println("ERROR: INA226 is NOT calibrated!");
    Serial.println("isCalibrated() returned false");
  } else {
    Serial.println("SUCCESS: INA226 is calibrated");
    Serial.print("Current LSB: ");
    Serial.print(ina226_5v.getCurrentLSB_uA(), 6);
    Serial.println(" uA");
    Serial.print("Shunt: ");
    Serial.print(ina226_5v.getShunt(), 4);
    Serial.println(" Ω");
    Serial.print("Max Current: ");
    Serial.print(ina226_5v.getMaxCurrent(), 4);
    Serial.println(" A");
  }
  ina226_5v.setModeShuntBusContinuous();

  
  uint16_t calibValue = readCalibrationRegister();
  Serial.print("Calibration register after setMaxCurrentShunt: 0x");
  Serial.println(calibValue, HEX);

  if (calibValue == 0) {
    Serial.println("ERROR: Calibration register is still zero!");
    Serial.println("The setMaxCurrentShunt method did not work!");
  } else {
    Serial.println("SUCCESS: Calibration register was set!");
  }

  // Set averaging and conversion times for better accuracy
  // INA226_1_SAMPLE, INA226_4_SAMPLES, INA226_16_SAMPLES, etc.
  ina226_5v.setAverage(INA226_16_SAMPLES);  // Average 16 samples
  
  // Set bus voltage conversion time (1100µs)
  ina226_5v.setBusVoltageConversionTime(INA226_1100_us);
  
  // Set shunt voltage conversion time (1100µs)  
  ina226_5v.setShuntVoltageConversionTime(INA226_1100_us);
  
  Serial.println("\n=== Bus Voltage Debug ===");

  // Read raw register directly
  uint16_t busRawReg = ina226_5v.getRegister(0x02);  // Bus voltage register
  Serial.print("Raw bus voltage register: 0x");
  Serial.print(busRawReg, HEX);
  Serial.print(" (");
  Serial.print(busRawReg);
  Serial.println(")");

  // INA226 bus voltage LSB = 1.25mV per bit
  float busVoltageFromRaw = busRawReg * 0.00125;  // 1.25mV per LSB
  Serial.print("Bus voltage from raw register: ");
  Serial.print(busVoltageFromRaw, 6);
  Serial.println(" V");

  // Now read via library
  float busVoltageLib = ina226_5v.getBusVoltage();
  Serial.print("Bus voltage from library: ");
  Serial.print(busVoltageLib, 6);
  Serial.println(" V");

  // Check if there's a scaling issue
  if (abs(busVoltageFromRaw - busVoltageLib) > 0.01) {
    Serial.println("⚠ WARNING: Library bus voltage doesn't match raw register!");
    Serial.println("   There may be a bus voltage scaling issue.");
  }
  // Check the configuration register
  uint16_t configReg = ina226_5v.getRegister(0x00);
  Serial.print("Config register: 0x");
  Serial.println(configReg, HEX);

  // Check the mode bits (should be 0x7 for continuous)
  uint8_t mode = configReg & 0x07;
  Serial.print("Operating mode: 0x");
  Serial.print(mode, HEX);
  if (mode == 0x07) {
    Serial.println(" ✓ (Continuous shunt and bus)");
  } else if (mode == 0x00) {
    Serial.println(" ✗ (POWER DOWN!)");
  } else {
    Serial.println(" ? (Unexpected mode)");
  }

  Serial.println("# ✓ INA226 (5V) initialized at 0x44 on Bus 1");
  Serial.println("#   Shunt: 0.1Ω (R100 confirmed)");
  Serial.println("#   Max current: 2A");
  Serial.println("#   Averaging: 16 samples");
  Serial.println("#   Calibrated with Rob Tillaart library");
  
  Serial.println("# ");
  Serial.println("# ⚠️  VOLTAGE DROP WARNING:");
  Serial.println("#   Two 0.1Ω shunts in series = 0.2Ω total");
  Serial.println("#   At 500mA: Voltage drop = 100mV");
  Serial.println("#   Servos will see: 5.0V - 0.1V = 4.9V");
  Serial.println("#   This should be acceptable for XL330 (3.7-6.0V range)");
  Serial.println("# ");
  
  // Reset statistics
  stats_ina219_5v.reset();
  stats_ina226_5v.reset();
  stats_diff.reset();
  
  Serial.println("# System ready!");
  Serial.println("# Tip: Use 'T' command to see 5V sensor agreement statistics");
  Serial.println("# ");
}

void loop() {
  handleSerialCommands();

  static uint32_t last_ms = 0;
  uint32_t now_ms = millis();

  if (!streaming) {
    delay(5);
    return;
  }

  if (now_ms - last_ms < sample_period_ms) {
    delay(1);
    return;
  }
  last_ms = now_ms;

  // ========== Read 12V rail sensor (Bus 0) ==========
  float bus12_v      = ina219_12v.getBusVoltage_V();
  float shunt12_mv   = ina219_12v.getShuntVoltage_mV();
  float current12_mA = ina219_12v.getCurrent_mA();
  float power12_mW   = ina219_12v.getPower_mW();

  float current12_A  = current12_mA / 1000.0f;
  float power12_W    = power12_mW   / 1000.0f;

  // Optional clamp if rail is off / very low
  if (bus12_v < 0.5f) {
    bus12_v    = 0.0f;
    power12_W  = 0.0f;
    current12_A = 0.0f;
  }

  // ========== Read 5V rail INA219 (Bus 0) ==========
  float bus5v_ina219_v      = ina219_5v.getBusVoltage_V();
  float shunt5v_ina219_mv   = ina219_5v.getShuntVoltage_mV();
  float current5v_ina219_mA = ina219_5v.getCurrent_mA();
  float power5v_ina219_mW   = ina219_5v.getPower_mW();

  float current5v_ina219_A  = current5v_ina219_mA / 1000.0f;
  float power5v_ina219_W    = power5v_ina219_mW   / 1000.0f;

  if (bus5v_ina219_v < 0.5f) {
    bus5v_ina219_v    = 0.0f;
    power5v_ina219_W  = 0.0f;
    current5v_ina219_A = 0.0f;
  }

  // ========== Read 5V rail INA226 (Bus 1) ==========
  // Rob Tillaart library functions
  float bus5v_ina226_v      = ina226_5v.getBusVoltage();           // V
  float shunt5v_ina226_mv   = ina226_5v.getShuntVoltage_mV();      // mV (already in mV!)
  float current5v_ina226_mA = ina226_5v.getCurrent_mA();           // mA (already in mA!)
  float power5v_ina226_mW   = ina226_5v.getPower_mW();             // mW (already in mW!)

  float orig_bus5v_ina226_v      = bus5v_ina226_v;        
  float orig_shunt5v_ina226_mv   = shunt5v_ina226_mv;     
  float orig_current5v_ina226_mA = current5v_ina226_mA;         
  float orig_power5v_ina226_mW   = power5v_ina226_mW;

  // WORKAROUND: Some INA226 modules don't return bus voltage correctly with this library
  // Only apply manual calculation if library returns zero for current but shunt is valid
  bool using_manual_calc = false;
  if (current5v_ina226_mA < 0.1 && shunt5v_ina226_mv > 1.0) {
    // Shunt voltage is valid but library isn't calculating current
    // Calculate current directly from shunt: I = V / R
    // R100 = 0.1Ω shunt resistor (confirmed on module)
    current5v_ina226_mA = shunt5v_ina226_mv / 0.1;  // mV / 0.1Ω = mA
    
    // NOTE: Bus voltage is estimated, not measured!
    // Estimated as INA219 voltage minus voltage drop across first shunt
    bus5v_ina226_v = bus5v_ina219_v - 0.05;  // Estimated (not true measurement)
    
    // Calculate power: P = V × I
    power5v_ina226_mW = bus5v_ina226_v * current5v_ina226_mA;
    
    using_manual_calc = true;
  }

  // Debug: Print first reading to show what's happening
  static bool first_read = true;
  if (first_read) {
    first_read = false;
    Serial.println("\n# === INA226 First Reading Debug ===");
    Serial.print("# Sensor Shunt Voltage: "); Serial.print(orig_shunt5v_ina226_mv, 6); Serial.println(" mV");
    Serial.print("# Sensor Current: "); Serial.print(orig_current5v_ina226_mA, 6); Serial.println(" mA");
    Serial.print("# Sensor Bus Voltage: "); Serial.print(orig_bus5v_ina226_v, 6); Serial.println(" V");
    Serial.print("# Sensor Power: "); Serial.print(orig_power5v_ina226_mW, 6); Serial.println(" mW");
    Serial.print("# Shunt Voltage: "); Serial.print(shunt5v_ina226_mv, 6); Serial.println(" mV");
    Serial.print("# Current: "); Serial.print(current5v_ina226_mA, 6); Serial.println(" mA");
    Serial.print("# Bus Voltage: "); Serial.print(bus5v_ina226_v, 6); Serial.println(" V");
    Serial.print("# Power: "); Serial.print(power5v_ina226_mW, 6); Serial.println(" mW");
    if (using_manual_calc) {
      Serial.println("# WARNING: Using manual calculation from shunt voltage");
      Serial.println("#          Bus voltage is ESTIMATED, not measured");
      Serial.println("#          Current comparison is still valid (uses shunt only)");
    } else {
      Serial.println("# Using library calculations (all values from INA226)");
    }
    Serial.println("# ===================================\n");
  }

  float current5v_ina226_A  = current5v_ina226_mA / 1000.0f;
  float power5v_ina226_W    = power5v_ina226_mW   / 1000.0f;

  if (bus5v_ina226_v < 0.5f) {
    bus5v_ina226_v    = 0.0f;
    power5v_ina226_W  = 0.0f;
    current5v_ina226_A = 0.0f;
  }

  // ========== Calculate 5V sensor comparison metrics ==========
  float current5v_diff_mA = (current5v_ina219_A - current5v_ina226_A) * 1000.0f;
  float current5v_avg_mA  = (current5v_ina219_A + current5v_ina226_A) / 2.0f * 1000.0f;

  // ========== Update statistics ==========
  stats_ina219_5v.add(current5v_ina219_A);
  stats_ina226_5v.add(current5v_ina226_A);
  stats_diff.add(current5v_ina219_A - current5v_ina226_A);

  uint32_t t_us = micros();

  // ========== Output CSV line ==========
  Serial.print(t_us);
  Serial.print(",");
  
  // 12V rail
  Serial.print(bus12_v,   4); Serial.print(",");
  Serial.print(shunt12_mv,4); Serial.print(",");
  Serial.print(current12_A,6); Serial.print(",");
  Serial.print(power12_W, 6); Serial.print(",");
  
  // 5V rail - INA219 (Bus 0)
  Serial.print(bus5v_ina219_v,    4); Serial.print(",");
  Serial.print(shunt5v_ina219_mv, 4); Serial.print(",");
  Serial.print(current5v_ina219_A,6); Serial.print(",");
  Serial.print(power5v_ina219_W,  6); Serial.print(",");
  
  // 5V rail - INA226 (Bus 1)
  Serial.print(bus5v_ina226_v,    4); Serial.print(",");
  Serial.print(shunt5v_ina226_mv, 4); Serial.print(",");
  Serial.print(current5v_ina226_A,6); Serial.print(",");
  Serial.print(power5v_ina226_W,  6); Serial.print(",");
  
  // 5V comparison
  Serial.print(current5v_diff_mA, 4); Serial.print(",");
  Serial.println(current5v_avg_mA, 4);
  
  // Auto-print statistics every 100 samples (optional - can remove if too verbose)
  static int sample_count = 0;
  sample_count++;
  if (sample_count >= 100) {
    printStats();
    sample_count = 0;
  }
}