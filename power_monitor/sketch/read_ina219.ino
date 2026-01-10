#include <Wire.h>
#include <Adafruit_INA219.h>

// ---------- I2C pins ----------
#define I2C_SDA 1
#define I2C_SCL 2

// ---------- Two INA219 sensors ----------
// 12V rail on address 0x40 (default)
Adafruit_INA219 ina219_12v(0x40);
// 5V rail on address 0x41 (second module, address jumper set!)
Adafruit_INA219 ina219_5v(0x41);

// ---------- Streaming control ----------
static bool     streaming        = true;   // start streaming by default
static uint32_t sample_period_ms = 100;    // 100 ms = 10 Hz

// ---------- Helpers ----------
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

    } else if (line.startsWith("P")) {
      int v = line.substring(1).toInt();
      if (v >= 10 && v <= 5000) {
        sample_period_ms = (uint32_t)v;
        Serial.print("# period_ms=");
        Serial.println(sample_period_ms);
      } else {
        Serial.println("# invalid period (10..5000 ms)");
      }

    } else if (line == "H") {
      Serial.println("# Commands:");
      Serial.println("#   S 1   -> start streaming");
      Serial.println("#   S 0   -> stop streaming");
      Serial.println("#   pNNN  -> set period in ms (e.g. p50, p100)");
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

  Serial.println("# Dual INA219 monitor (12V + 5V rails)");
  Serial.println("# CSV columns:");
  Serial.println("# t_us,"
                 "bus12_v,shunt12_mv,current12_A,power12_W,"
                 "bus5_v,shunt5_mv,current5_A,power5_W");
  Serial.println("# Commands: 'S 1', 'S 0', 'pNNN', 'H'");

  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize 12V rail sensor
  if (!ina219_12v.begin()) {
    Serial.println("# ERROR: Failed to find INA219 (12V) at 0x40");
    while (1) {
      delay(500);
      Serial.println("# still failing ina219_12v.begin()");
    }
  }

  // Initialize 5V rail sensor
  if (!ina219_5v.begin()) {
    Serial.println("# ERROR: Failed to find INA219 (5V) at 0x41");
    while (1) {
      delay(500);
      Serial.println("# still failing ina219_5v.begin()");
    }
  }

  // Calibration: 32V bus, 2A range is a good general setting for both rails
  ina219_12v.setCalibration_32V_2A();
  ina219_5v.setCalibration_32V_2A();

  Serial.println("# INA219 12V and 5V sensors initialized.");
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

  // ---- Read 12V rail sensor ----
  float bus12_v      = ina219_12v.getBusVoltage_V();
  float shunt12_mv   = ina219_12v.getShuntVoltage_mV();
  float current12_mA = ina219_12v.getCurrent_mA();
  float power12_mW   = ina219_12v.getPower_mW();

  float current12_A  = current12_mA / 1000.0f;
  float power12_W    = power12_mW   / 1000.0f;

  // Optional clamp if rail is off / very low
  if (bus12_v < 0.5f) {
    bus12_v   = 0.0f;
    power12_W = 0.0f;
  }

  // ---- Read 5V rail sensor ----
  float bus5_v      = ina219_5v.getBusVoltage_V();
  float shunt5_mv   = ina219_5v.getShuntVoltage_mV();
  float current5_mA = ina219_5v.getCurrent_mA();
  float power5_mW   = ina219_5v.getPower_mW();

  float current5_A  = current5_mA / 1000.0f;
  float power5_W    = power5_mW   / 1000.0f;

  if (bus5_v < 0.5f) {
    bus5_v   = 0.0f;
    power5_W = 0.0f;
  }

  uint32_t t_us = micros();

  // ---- Single CSV line with both rails ----
  Serial.print(t_us);
  Serial.print(",");
  Serial.print(bus12_v,   4); Serial.print(",");
  Serial.print(shunt12_mv,4); Serial.print(",");
  Serial.print(current12_A,6); Serial.print(",");
  Serial.print(power12_W, 6); Serial.print(",");
  Serial.print(bus5_v,    4); Serial.print(",");
  Serial.print(shunt5_mv, 4); Serial.print(",");
  Serial.print(current5_A,6); Serial.print(",");
  Serial.println(power5_W,6);
}
