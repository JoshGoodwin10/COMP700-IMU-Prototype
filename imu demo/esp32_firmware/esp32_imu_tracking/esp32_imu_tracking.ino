/*
  IMU Biomechanics Lab — ESP32 Firmware
  Hardware : ESP32 + HW-131 (MPU-6050)
  Library  : MPU6050 by Electronic Cats  (Sketch > Include Library > Manage Libraries)

  Wiring:
    HW-131 VCC → ESP32 3.3V
    HW-131 GND → ESP32 GND
    HW-131 SCL → ESP32 GPIO 22
    HW-131 SDA → ESP32 GPIO 21

  On boot the sensor runs an automatic calibration routine (≈ 5 s).
  Keep the sensor flat and completely still during that time.
  Calibration offsets are applied in-firmware and reported to the dashboard.

  Serial output (115200 baud):
    Calibration progress: {"cal":1}  …  {"cal":9}
    Calibration done    : {"cal":"done","ax_off":…,"ay_off":…,…}
    Live data           : {"r":12.3,"p":-5.1,"y":33.2,"ax":0.12,"ay":-0.03,"az":0.99,"t":1234}
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ── Calibration settings ───────────────────────────────────────
// How many samples to average per calibration step (more = more accurate, slower)
const int   CAL_SAMPLES  = 1000;
// Number of calibration passes (each pass refines the offsets)
const int   CAL_PASSES   = 10;

// Discovered offsets — filled during calibration, applied before streaming
int16_t ax_off = 0, ay_off = 0, az_off = 0;
int16_t gx_off = 0, gy_off = 0, gz_off = 0;

// ── Orientation state ──────────────────────────────────────────
float roll  = 0.0;
float pitch = 0.0;
float yaw   = 0.0;
float ax_g  = 0.0, ay_g = 0.0, az_g = 0.0;

// ── Timing ────────────────────────────────────────────────────
unsigned long lastTime       = 0;
const int     SAMPLE_RATE_HZ = 100;
const int     SAMPLE_MS      = 1000 / SAMPLE_RATE_HZ;

// Complementary filter coefficient
const float ALPHA = 0.96;

// ── Scale factors (set after mpu.setFullScale… calls) ─────────
// Accel ±4 g  → 8192 LSB/g
// Gyro  ±500°/s → 65.5 LSB/(°/s)
const float ACCEL_SCALE = 8192.0;
const float GYRO_SCALE  = 65.5;

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);       // SDA=21, SCL=22

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("{\"error\":\"MPU6050 not found — check wiring\"}");
    while (true) delay(500);
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);   // ±4 g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500 °/s
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // 42 Hz LPF

  Serial.println("{\"status\":\"calibrating\",\"msg\":\"Keep sensor flat and still\"}");
  delay(500);   // brief settle time before sampling

  calibrate();

  // Apply offsets discovered during calibration
  mpu.setXAccelOffset(ax_off);
  mpu.setYAccelOffset(ay_off);
  mpu.setZAccelOffset(az_off);
  mpu.setXGyroOffset(gx_off);
  mpu.setYGyroOffset(gy_off);
  mpu.setZGyroOffset(gz_off);

  // Report final offsets to dashboard
  Serial.print("{\"cal\":\"done\"");
  Serial.print(",\"ax_off\":"); Serial.print(ax_off);
  Serial.print(",\"ay_off\":"); Serial.print(ay_off);
  Serial.print(",\"az_off\":"); Serial.print(az_off);
  Serial.print(",\"gx_off\":"); Serial.print(gx_off);
  Serial.print(",\"gy_off\":"); Serial.print(gy_off);
  Serial.print(",\"gz_off\":"); Serial.print(gz_off);
  Serial.println("}");

  delay(200);
  lastTime = millis();
}

// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  if (now - lastTime < SAMPLE_MS) return;
  float dt  = (now - lastTime) / 1000.0;
  lastTime  = now;

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  ax_g = rawAx / ACCEL_SCALE;
  ay_g = rawAy / ACCEL_SCALE;
  az_g = rawAz / ACCEL_SCALE;

  float gx = rawGx / GYRO_SCALE;
  float gy = rawGy / GYRO_SCALE;
  float gz = rawGz / GYRO_SCALE;

  // Accel-derived angles
  float accel_roll  = atan2(ay_g, az_g)                                          * 180.0 / PI;
  float accel_pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g))              * 180.0 / PI;

  // Complementary filter
  roll  = ALPHA * (roll  + gx * dt) + (1.0 - ALPHA) * accel_roll;
  pitch = ALPHA * (pitch + gy * dt) + (1.0 - ALPHA) * accel_pitch;
  yaw  += gz * dt;   // gyro-only; drifts slowly without magnetometer

  if (yaw >  180.0) yaw -= 360.0;
  if (yaw < -180.0) yaw += 360.0;

  // Compact JSON — one line per sample
  Serial.print("{\"r\":");  Serial.print(roll,  2);
  Serial.print(",\"p\":"); Serial.print(pitch, 2);
  Serial.print(",\"y\":"); Serial.print(yaw,   2);
  Serial.print(",\"ax\":"); Serial.print(ax_g,  4);
  Serial.print(",\"ay\":"); Serial.print(ay_g,  4);
  Serial.print(",\"az\":"); Serial.print(az_g,  4);
  Serial.print(",\"t\":"); Serial.print(now);
  Serial.println("}");
}

// ─────────────────────────────────────────────────────────────
// calibrate()
//
// Runs CAL_PASSES refinement loops, each averaging CAL_SAMPLES
// readings. After each pass it adjusts the six hardware offsets
// so the sensor converges toward:
//   ax=0, ay=0, az=+1g   (flat on table, Z pointing up)
//   gx=0, gy=0, gz=0     (stationary)
//
// Progress messages are sent as {"cal":1} … {"cal":9} so the
// dashboard can show a progress bar.
// ─────────────────────────────────────────────────────────────
void calibrate() {
  long sumAx, sumAy, sumAz, sumGx, sumGy, sumGz;

  // Start from zero offsets for a clean baseline
  mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);
  ax_off = 0; ay_off = 0; az_off = 0;
  gx_off = 0; gy_off = 0; gz_off = 0;

  for (int pass = 0; pass < CAL_PASSES; pass++) {

    // Send progress (1–9, skip 0 to avoid confusion with false)
    if (pass > 0) {
      Serial.print("{\"cal\":"); Serial.print(pass); Serial.println("}");
    }

    sumAx = 0; sumAy = 0; sumAz = 0;
    sumGx = 0; sumGy = 0; sumGz = 0;

    for (int i = 0; i < CAL_SAMPLES; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      sumAx += ax; sumAy += ay; sumAz += az;
      sumGx += gx; sumGy += gy; sumGz += gz;
      delayMicroseconds(100);   // ~10 kHz internal sample rate, well above our 100 Hz
    }

    // Mean readings
    int16_t meanAx = sumAx / CAL_SAMPLES;
    int16_t meanAy = sumAy / CAL_SAMPLES;
    int16_t meanAz = sumAz / CAL_SAMPLES;
    int16_t meanGx = sumGx / CAL_SAMPLES;
    int16_t meanGy = sumGy / CAL_SAMPLES;
    int16_t meanGz = sumGz / CAL_SAMPLES;

    // Target: accel reads (0, 0, +1g) = (0, 0, +8192) in ±4g range
    // Error = mean - target; subtract from offset to push toward target
    ax_off -= meanAx / 8;           // /8 scales from accel LSB to offset register units
    ay_off -= meanAy / 8;
    az_off -= (meanAz - (int16_t)ACCEL_SCALE) / 8;   // subtract 1 g from Z target

    gx_off -= meanGx / 4;           // gyro registers have ~4× sensitivity vs raw
    gy_off -= meanGy / 4;
    gz_off -= meanGz / 4;

    // Apply updated offsets for next pass
    mpu.setXAccelOffset(ax_off); mpu.setYAccelOffset(ay_off); mpu.setZAccelOffset(az_off);
    mpu.setXGyroOffset(gx_off);  mpu.setYGyroOffset(gy_off);  mpu.setZGyroOffset(gz_off);
  }
}
