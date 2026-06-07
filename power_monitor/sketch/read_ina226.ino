// read_ina226.ino
// Four-sensor CSV power monitor — INA226 sensors only.
//
// Hardware:
//   I2C (GPIO21=SDA, GPIO47=SCL): INA226 12V @ 0x40, INA226 5V @ 0x44
//
// CSV column layout (19 cols, matches power_monitor_node.py):
//   col  0: t_us
//   col  1- 4: INA219 12V  -- always zero (sensor not present)
//   col  5- 8: INA219 5V   -- always zero (sensor not present)
//   col  9-12: INA226 12V  (bus_v, shunt_mv, current_A, power_W)
//   col 13-16: INA226 5V   (bus_v, shunt_mv, current_A, power_W)
//   col 17:    current_5v_diff_mA  -- always zero (no cross-check sensor)
//   col 18:    current_5v_avg_mA   -- always zero (no cross-check sensor)

#include <Wire.h>
#include <INA226.h>  // Rob Tillaart library

#define I2C_SDA 21
#define I2C_SCL 47

#define ADDR_12V 0x40
#define ADDR_5V  0x44

INA226 ina226_12v(ADDR_12V, &Wire);
INA226 ina226_5v(ADDR_5V,   &Wire);


// ---------- Streaming control ----------
static bool     streaming        = true;
static uint32_t sample_period_ms = 100;   // 10 Hz default

// ---------- Helpers ----------

// Scan the I2C bus and print all responding addresses.
// Returns true if both expected addresses are present.
bool scanBus() {
  Serial.println("# --- I2C bus scan ---");
  int found = 0;
  bool has_12v = false;
  bool has_5v  = false;

  for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("#   0x"); Serial.print(addr, HEX);
      if      (addr == ADDR_12V) { Serial.print("  <- INA226 12V (expected)"); has_12v = true; }
      else if (addr == ADDR_5V)  { Serial.print("  <- INA226 5V  (expected)"); has_5v  = true; }
      else                       { Serial.print("  (unexpected device)"); }
      Serial.println();
      found++;
    }
  }

  if (found == 0) {
    Serial.println("# WARNING: no devices found on bus -- check wiring");
  }
  if (!has_12v) {
    Serial.print("# ERROR: INA226 12V not found at 0x"); Serial.println(ADDR_12V, HEX);
  }
  if (!has_5v) {
    Serial.print("# ERROR: INA226 5V not found at 0x");  Serial.println(ADDR_5V,  HEX);
  }
  if (has_12v && has_5v) {
    Serial.println("# Both sensors present -- proceeding with init");
  }
  Serial.println("# --------------------");
  return has_12v && has_5v;
}

// Read calibration register directly (for debug)
uint16_t readCalibrationRegister(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x05);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0xFFFF;
}

// Initialise one INA226 and print diagnostics.
// Returns true if calibration succeeded.
bool initINA226(INA226 &sensor, uint8_t addr, const char *label,
                float max_current_A, float shunt_ohm) {
  Serial.print("# Initializing INA226 ("); Serial.print(label);
  Serial.print(") at 0x"); Serial.print(addr, HEX); Serial.println("...");

  if (!sensor.begin()) {
    Serial.print("# ERROR: INA226 ("); Serial.print(label);
    Serial.println(") begin() failed -- check wiring and address");
    return false;
  }

  int result = sensor.setMaxCurrentShunt(max_current_A, shunt_ohm);
  if (result != INA226_ERR_NONE) {
    Serial.print("# ERROR: calibration failed for INA226 ("); Serial.print(label);
    Serial.print("), code="); Serial.println(result);
    return false;
  }

  if (!sensor.isCalibrated()) {
    Serial.print("# ERROR: INA226 ("); Serial.print(label);
    Serial.println(") not calibrated after setMaxCurrentShunt");
    return false;
  }

  Serial.print("#   current LSB: "); Serial.print(sensor.getCurrentLSB_uA(), 4); Serial.println(" uA");
  Serial.print("#   shunt:       "); Serial.print(sensor.getShunt(), 4);          Serial.println(" ohm");
  Serial.print("#   max current: "); Serial.print(sensor.getMaxCurrent(), 4);     Serial.println(" A");

  sensor.setModeShuntBusContinuous();
  sensor.setAverage(INA226_16_SAMPLES);
  sensor.setBusVoltageConversionTime(INA226_1100_us);
  sensor.setShuntVoltageConversionTime(INA226_1100_us);

  uint16_t calReg = readCalibrationRegister(addr);
  Serial.print("#   calibration register: 0x"); Serial.println(calReg, HEX);
  if (calReg == 0) {
    Serial.println("# ERROR: calibration register still zero");
    return false;
  }

  Serial.print("# INA226 ("); Serial.print(label); Serial.println(") ready");
  return true;
}

// Read one INA226 with manual-calculation workaround.
// Some modules return zero current despite valid shunt voltage (library bug).
// Assumes R100 shunt (0.1 ohm): I(mA) = V_shunt(mV) / 0.1
// fallback_bus_v is used for bus voltage when the workaround fires.
void readINA226(INA226 &sensor, float fallback_bus_v,
                float &out_bus_v, float &out_shunt_mv,
                float &out_current_A, float &out_power_W) {
  out_bus_v    = sensor.getBusVoltage();
  out_shunt_mv = sensor.getShuntVoltage_mV();
  float cur_mA = sensor.getCurrent_mA();
  float pow_mW = sensor.getPower_mW();

  if (cur_mA < 0.1f && out_shunt_mv > 1.0f) {
    cur_mA    = out_shunt_mv / 0.1f;
    out_bus_v = fallback_bus_v;
    pow_mW    = out_bus_v * cur_mA;
  }

  out_current_A = cur_mA / 1000.0f;
  out_power_W   = pow_mW / 1000.0f;

  if (out_bus_v < 0.5f) {
    out_bus_v     = 0.0f;
    out_current_A = 0.0f;
    out_power_W   = 0.0f;
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    line.toUpperCase();

    if (line.startsWith("S")) {
      int val = line.length() >= 2 ? line.substring(1).toInt() : 0;
      streaming = (val != 0);
      Serial.print("# streaming="); Serial.println(streaming ? "1" : "0");

    } else if (line.startsWith("P")) {
      int v = line.substring(1).toInt();
      if (v >= 10 && v <= 5000) {
        sample_period_ms = (uint32_t)v;
        Serial.print("# period_ms="); Serial.println(sample_period_ms);
      } else {
        Serial.println("# invalid period (10..5000 ms)");
      }

    } else if (line == "H") {
      Serial.println("# Commands:");
      Serial.println("#   S 1   -> start streaming");
      Serial.println("#   S 0   -> stop streaming");
      Serial.println("#   Pnnn  -> set period ms (10-5000)");
      Serial.println("#   H     -> this help");

    } else {
      Serial.print("# unknown cmd: "); Serial.println(line);
    }
  }
}

// ---------- setup ----------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("# INA226 Power Monitor (ESP32-S3)");
  Serial.println("# I2C (GPIO21=SDA, GPIO47=SCL): INA226 12V @ 0x40, INA226 5V @ 0x44");
  Serial.println("# CSV columns:");
  Serial.println("# t_us,"
                 "bus12_219_v,shunt12_219_mv,current12_219_A,power12_219_W,"   // zeros
                 "bus5_219_v,shunt5_219_mv,current5_219_A,power5_219_W,"        // zeros
                 "bus12_226_v,shunt12_226_mv,current12_226_A,power12_226_W,"
                 "bus5_226_v,shunt5_226_mv,current5_226_A,power5_226_W,"
                 "current5v_diff_mA,current5v_avg_mA");                         // zeros

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(1500);   // let sensors power up before any I2C traffic

  // Scan bus first — confirms addresses before init attempts
  scanBus();

  // 12V sensor: 0.1 ohm shunt, max 2.0A
  initINA226(ina226_12v, ADDR_12V, "12V", 0.800f, 0.1f);

  // 5V sensor: 0.1 ohm shunt, max 0.8A
  initINA226(ina226_5v,  ADDR_5V,  "5V",  0.800f, 0.1f);

  Serial.println("# ready -- streaming at 10 Hz");
  Serial.println("# ");
}

// ---------- loop ----------

void loop() {
  handleSerialCommands();

  static uint32_t last_ms = 0;
  uint32_t now_ms = millis();

  if (!streaming) { delay(5); return; }
  if (now_ms - last_ms < sample_period_ms) { delay(1); return; }
  last_ms = now_ms;

  // ---- INA226 12V ----
  // Fallback bus voltage: nominal 12V minus a small shunt drop estimate
  float bus12_226_v, shunt12_226_mv, current12_226_A, power12_226_W;
  readINA226(ina226_12v, 11.95f,
             bus12_226_v, shunt12_226_mv, current12_226_A, power12_226_W);

  // ---- INA226 5V ----
  float bus5_226_v, shunt5_226_mv, current5_226_A, power5_226_W;
  readINA226(ina226_5v, 4.95f,
             bus5_226_v, shunt5_226_mv, current5_226_A, power5_226_W);

  // ---- First-read debug ----
  static bool first_read = true;
  if (first_read) {
    first_read = false;
    Serial.println("# === First Reading ===");
    Serial.print("# INA226 12V: bus_v="); Serial.print(bus12_226_v, 4);
    Serial.print(" V  current=");          Serial.print(current12_226_A * 1000, 3); Serial.println(" mA");
    Serial.print("# INA226 5V:  bus_v="); Serial.print(bus5_226_v, 4);
    Serial.print(" V  current=");          Serial.print(current5_226_A * 1000, 3); Serial.println(" mA");
    Serial.println("# =====================");
  }

  // ---- CSV output ----
  uint32_t t_us = micros();
  Serial.print(t_us); Serial.print(",");

  // INA219 columns -- zero (sensor not present)
  Serial.print("0,0,0,0,");   // 12V INA219
  Serial.print("0,0,0,0,");   // 5V  INA219

  // INA226 12V
  Serial.print(bus12_226_v,     4); Serial.print(",");
  Serial.print(shunt12_226_mv,  4); Serial.print(",");
  Serial.print(current12_226_A, 6); Serial.print(",");
  Serial.print(power12_226_W,   6); Serial.print(",");

  // INA226 5V
  Serial.print(bus5_226_v,     4); Serial.print(",");
  Serial.print(shunt5_226_mv,  4); Serial.print(",");
  Serial.print(current5_226_A, 6); Serial.print(",");
  Serial.print(power5_226_W,   6); Serial.print(",");

  // Cross-check columns -- zero (no second sensor to compare)
  Serial.print("0,");
  Serial.println("0");
}
