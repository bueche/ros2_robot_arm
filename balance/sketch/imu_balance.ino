/**
 * mpu6050_balance_imu.ino
 *
 * ESP32 + MPU-6050 (GY-521) — Cup/Ball Bearing Balance IMU
 * 
 * Hardware:
 *   ESP32-D0WD-V3 (dual core)
 *   MPU-6050 on GY-521 breakout, connected via I2C
 *
 * Wiring (GY-521 → ESP32):
 *   VCC  → 3.3V  (the GY-521 has an onboard 3.3V regulator, so 5V also works)
 *   GND  → GND
 *   SCL  → GPIO 22  (default ESP32 I2C SCL)
 *   SDA  → GPIO 21  (default ESP32 I2C SDA)
 *   AD0  → GND      (sets I2C address to 0x68; pull HIGH for 0x69)
 *   INT  → GPIO 4   (optional — used for DMP data-ready interrupt if you enable DMP later)
 *
 * Libraries (install via Arduino Library Manager):
 *   - "MPU6050" by Electronic Cats  (search: MPU6050 Electronic Cats)
 *   - "Adafruit Unified Sensor"     (dependency, install if prompted)
 *
 * Output (Serial, 115200 baud):
 *   JSON lines on Serial, one per sample:
 *   {"pitch":0.12,"roll":-0.05,"dp":0.003,"dr":-0.001,"ax":0.01,"ay":-0.02,"az":9.79,"temp":28.4,"ts":12345}
 *
 *   Fields:
 *     pitch  — tilt around the wrist-flex axis (radians, calibration-zeroed)
 *     roll   — tilt around the wrist-roll axis (radians, calibration-zeroed)
 *     dp     — pitch rate (rad/s, from gyro)
 *     dr     — roll rate  (rad/s, from gyro)
 *     ax,ay,az — calibrated acceleration (m/s², mostly useful for diagnostics)
 *     temp   — chip temperature (°C)
 *     ts     — millis() timestamp
 *
 * Calibration:
 *   On startup (or on 'c' command over Serial) the sketch averages N samples
 *   while the arm is in the "balanced/level" pose and stores those as offsets.
 *   All subsequent readings are offset-subtracted before publishing.
 *
 * Commands (send over Serial):
 *   'c'  — recalibrate (zero at current orientation)
 *   'r'  — report raw (un-calibrated) values for one second
 *   's'  — toggle streaming on/off
 *
 * ROS2 Integration:
 *   The ROS2 node (imu_balance_node.py) reads this JSON stream over a serial
 *   port and publishes sensor_msgs/Imu + custom balance error topics.
 *   See companion file: imu_balance_node.py
 *
 * Complementary Filter:
 *   Pure gyro integration drifts; pure accelerometer is noisy.
 *   We fuse them with a simple complementary filter:
 *     angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
 *   alpha = 0.98 is a good starting point (trusts gyro for fast motion,
 *   accelerometer for long-term drift correction).
 *   Tune COMP_ALPHA if the response feels sluggish (lower) or noisy (higher).
 */

#include <Wire.h>
#include <MPU6050.h>

// ── Pin / I2C config ──────────────────────────────────────────────────────
#define SDA_PIN       21
#define SCL_PIN       22
#define INT_PIN        4   // not used in this sketch, wired for future DMP use
#define I2C_FREQ   400000  // 400 kHz fast-mode

// ── Sampling ──────────────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ   50          // target publish rate
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ)

// ── Calibration ───────────────────────────────────────────────────────────
#define CAL_SAMPLES     200           // averages over this many samples (~5s at 50Hz)

// ── Complementary filter ──────────────────────────────────────────────────
// Higher = trust gyro more (less accel noise, more drift)
// Lower  = trust accel more (less drift, more vibration noise)
#define COMP_ALPHA     0.98f

// ── Gravity constant ──────────────────────────────────────────────────────
#define G_MS2          9.80665f       // m/s²

// ── Scale factors (MPU-6050 raw → SI units) ───────────────────────────────
// Accel full-scale ±2g  → 16384 LSB/g
// Gyro  full-scale ±250°/s → 131 LSB/(°/s)
#define ACCEL_SCALE    (G_MS2 / 16384.0f)
#define GYRO_SCALE     (1.0f / 131.0f * DEG_TO_RAD)   // → rad/s

// ═════════════════════════════════════════════════════════════════════════
MPU6050 mpu;

// Calibration offsets (in raw LSB units — subtracted before scaling)
struct CalOffsets {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} cal = {0, 0, 0, 0, 0, 0};

// Complementary filter state
float pitch = 0.0f;   // rad, wrist-flex axis
float roll  = 0.0f;   // rad, wrist-roll axis

// Control flags
bool streaming = true;

// Timing
unsigned long lastSampleUs = 0;

// ─── Forward declarations ─────────────────────────────────────────────────
void calibrate();
void readAndPublish();
void handleSerial();
void printRaw();

void setup() {
  Serial.begin(115200);
  Serial.println("starting...");
  Serial.flush();
  // Native USB CDC (usbmodem) needs extra time to enumerate on the host
  // before prints are visible. Wait up to 3s, then proceed regardless.
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { delay(10); }
  delay(2000);

  Serial.println("# MPU-6050 Balance IMU - ESP32-WROOM-32D");
  Serial.println("# Commands: c=calibrate  r=raw dump  s=toggle stream");
  Serial.flush();

  // I2C init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // MPU-6050 init
  Serial.println("about to initialize mpu");
  Serial.flush();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("# ERROR: MPU-6050 not found. Check wiring (SDA=21, SCL=22).");
    Serial.flush();
    while (true) { delay(1000); }
  }
  Serial.println("# MPU-6050 found on I2C.");
  Serial.flush();

  // Set full-scale ranges
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // +-2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // +-250 deg/s

  // Low-pass filter: 42Hz cutoff
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  // Sample rate divider: 1kHz / (1 + 9) = 100Hz
  mpu.setRate(9);

  Serial.println("# MPU-6050 configured. Starting calibration...");
  Serial.flush();
  calibrate();

  Serial.println("# Streaming started. Recalibrate anytime with 'c'.");
  Serial.flush();
  lastSampleUs = micros();
}

// ═════════════════════════════════════════════════════════════════════════
void loop() {
  // Non-blocking sample timer (avoids delay() blocking serial handling)
  unsigned long now = micros();
  if (streaming && (now - lastSampleUs >= SAMPLE_INTERVAL_US)) {
    lastSampleUs = now;
    readAndPublish();
  }

  handleSerial();
}

// ═════════════════════════════════════════════════════════════════════════
/**
 * Calibrate: average CAL_SAMPLES readings and store as zero-offsets.
 * Call this with the cup attached and the arm in the "level/balanced" pose.
 */
void calibrate() {
  Serial.println("# Calibrating - hold sensor steady...");
  Serial.flush();

  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;
  int16_t ax, ay, az, gx, gy, gz;

  // Discard first 50 samples using millis timing (non-blocking friendly)
  for (int i = 0; i < 50; i++) {
    unsigned long t = millis();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    while (millis() - t < 5) { yield(); }  // ~5ms between samples, yield for watchdog
  }

  for (int i = 0; i < CAL_SAMPLES; i++) {
    unsigned long t = millis();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAx += ax;  sumAy += ay;  sumAz += az;
    sumGx += gx;  sumGy += gy;  sumGz += gz;
    while (millis() - t < 5) { yield(); }  // yield keeps watchdog happy

    // Print progress every 100 samples so monitor shows activity
    if ((i + 1) % 100 == 0) {
      Serial.print("# Cal progress: ");
      Serial.print(i + 1);
      Serial.print("/");
      Serial.println(CAL_SAMPLES);
      Serial.flush();
    }
  }

  cal.ax = sumAx / CAL_SAMPLES;
  cal.ay = sumAy / CAL_SAMPLES;
  cal.az = sumAz / CAL_SAMPLES;
  cal.gx = sumGx / CAL_SAMPLES;
  cal.gy = sumGy / CAL_SAMPLES;
  cal.gz = sumGz / CAL_SAMPLES;

  // Note: az offset should not zero out gravity — it should zero the TILT.
  // The gravity component is naturally removed by the atan2 math, but
  // the az DC offset correction still helps accuracy.

  // Reset filter state
  pitch = 0.0f;
  roll  = 0.0f;

  Serial.print("# Cal offsets — ax:");  Serial.print(cal.ax);
  Serial.print(" ay:");                  Serial.print(cal.ay);
  Serial.print(" az:");                  Serial.print(cal.az);
  Serial.print(" gx:");                  Serial.print(cal.gx);
  Serial.print(" gy:");                  Serial.print(cal.gy);
  Serial.print(" gz:");                  Serial.println(cal.gz);
  Serial.println("# Calibration complete.");
  Serial.flush();
}

// ═════════════════════════════════════════════════════════════════════════
/**
 * Read one sample, run complementary filter, publish JSON.
 *
 * Axis mapping for the GY-521 mounted on the cup/hand link:
 *   Pitch (wrist_flex axis) — forward/back tilt  → driven by accel X / gyro Y
 *   Roll  (wrist_roll axis) — left/right tilt    → driven by accel Y / gyro X
 *
 * If your physical mounting rotates these, swap the axis assignments here
 * rather than in the ROS node — keep the math close to the sensor.
 */
void readAndPublish() {
  static unsigned long lastUs = 0;
  int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

  mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

  // Apply calibration offsets
  float ax = (ax_r - cal.ax) * ACCEL_SCALE;
  float ay = (ay_r - cal.ay) * ACCEL_SCALE;
  float az = (az_r - cal.az) * ACCEL_SCALE + G_MS2;  // restore gravity on z
  float gx = (gx_r - cal.gx) * GYRO_SCALE;           // rad/s
  float gy = (gy_r - cal.gy) * GYRO_SCALE;
  float gz = (gz_r - cal.gz) * GYRO_SCALE;

  // dt in seconds
  unsigned long nowUs = micros();
  float dt = (lastUs == 0) ? (1.0f / SAMPLE_RATE_HZ) : ((nowUs - lastUs) * 1e-6f);
  lastUs = nowUs;
  dt = constrain(dt, 0.001f, 0.1f);   // guard against timer glitches

  // Accelerometer-derived tilt angles (noisy but drift-free)
  float accel_pitch = atan2f(ax, sqrtf(ay*ay + az*az));
  float accel_roll  = atan2f(-ay, az);

  // Complementary filter fusion
  pitch = COMP_ALPHA * (pitch + gy * dt) + (1.0f - COMP_ALPHA) * accel_pitch;
  roll  = COMP_ALPHA * (roll  + gx * dt) + (1.0f - COMP_ALPHA) * accel_roll;

  // Chip temperature (register 41, formula from datasheet)
  float temp_c = mpu.getTemperature() / 340.0f + 36.53f;

  // Publish as JSON — one line per sample
  // The ROS2 node parses this stream and publishes sensor_msgs/Imu
  Serial.print("{");
  Serial.print("\"pitch\":");   Serial.print(pitch,  5);
  Serial.print(",\"roll\":");   Serial.print(roll,   5);
  Serial.print(",\"dp\":");     Serial.print(gy,     5);   // pitch rate
  Serial.print(",\"dr\":");     Serial.print(gx,     5);   // roll rate
  Serial.print(",\"ax\":");     Serial.print(ax,     4);
  Serial.print(",\"ay\":");     Serial.print(ay,     4);
  Serial.print(",\"az\":");     Serial.print(az,     4);
  Serial.print(",\"temp\":");   Serial.print(temp_c, 2);
  Serial.print(",\"ts\":");     Serial.print(millis());
  Serial.println("}");
}

// ═════════════════════════════════════════════════════════════════════════
void handleSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  switch (cmd) {
    case 'c':
    case 'C':
      calibrate();
      break;

    case 'r':
    case 'R':
      printRaw();
      break;

    case 's':
    case 'S':
      streaming = !streaming;
      Serial.print("# Streaming: ");
      Serial.println(streaming ? "ON" : "OFF");
      break;

    default:
      break;
  }
}

// ═════════════════════════════════════════════════════════════════════════
/**
 * Dump raw (uncalibrated) values for 1 second — useful for checking
 * sensor orientation and verifying axis directions before calibration.
 */
void printRaw() {
  Serial.println("# RAW dump (1 second, uncalibrated):");
  bool was_streaming = streaming;
  streaming = false;
  unsigned long start = millis();

  while (millis() - start < 1000) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("# RAW ax="); Serial.print(ax);
    Serial.print(" ay=");      Serial.print(ay);
    Serial.print(" az=");      Serial.print(az);
    Serial.print(" gx=");      Serial.print(gx);
    Serial.print(" gy=");      Serial.print(gy);
    Serial.print(" gz=");      Serial.println(gz);
    Serial.flush();
    unsigned long t = millis();
    while (millis() - t < 50) { yield(); }
  }

  streaming = was_streaming;
  Serial.println("# RAW dump done.");
}
