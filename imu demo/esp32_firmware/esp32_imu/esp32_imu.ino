/*
  IMU Biomechanics Lab — ESP32 Firmware
  Hardware: ESP32 + HW-131 (MPU-6050)
  
  Wiring:
    HW-131 VCC  → ESP32 3.3V
    HW-131 GND  → ESP32 GND
    HW-131 SCL  → ESP32 GPIO 22
    HW-131 SDA  → ESP32 GPIO 21
    HW-131 INT  → (optional) ESP32 GPIO 19

  Library required: MPU6050 by Electronic Cats (install via Arduino Library Manager)
  
  Output format (Serial, 115200 baud):
    JSON line per sample: {"r":12.3,"p":-5.1,"y":33.2,"ax":0.12,"ay":-0.03,"az":0.99,"t":1234}
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Calibration offsets (run calibration sketch first, paste values here)
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Orientation state
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float ax_g = 0.0, ay_g = 0.0, az_g = 0.0;

// Timing
unsigned long lastTime = 0;
const int SAMPLE_RATE_HZ = 100;
const int SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;

// Complementary filter coefficient (0.98 = trust gyro more, 0.02 = trust accel)
const float ALPHA = 0.96;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA=21, SCL=22 (ESP32 defaults)
  
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("{\"error\":\"MPU6050 not found. Check wiring.\"}");
    while (1) delay(500);
  }

  // Set ranges
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // ±4g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // ±500 deg/s
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);             // 42 Hz low-pass filter

  // Apply calibration offsets
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  Serial.println("{\"status\":\"ready\",\"rate\":100}");
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime < SAMPLE_INTERVAL_MS) return;
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  // Convert to physical units
  // Accel: ±4g range → 8192 LSB/g
  ax_g = rawAx / 8192.0;
  ay_g = rawAy / 8192.0;
  az_g = rawAz / 8192.0;

  // Gyro: ±500 deg/s range → 65.5 LSB/(deg/s)
  float gx = rawGx / 65.5;
  float gy = rawGy / 65.5;
  float gz = rawGz / 65.5;

  // Accel-based roll and pitch (good when near-static)
  float accel_roll  = atan2(ay_g, az_g) * 180.0 / PI;
  float accel_pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

  // Complementary filter: blend gyro integration with accel correction
  roll  = ALPHA * (roll  + gx * dt) + (1.0 - ALPHA) * accel_roll;
  pitch = ALPHA * (pitch + gy * dt) + (1.0 - ALPHA) * accel_pitch;
  yaw  += gz * dt;  // Gyro only (no magnetometer for yaw correction)

  // Clamp yaw to ±180
  if (yaw > 180.0)  yaw -= 360.0;
  if (yaw < -180.0) yaw += 360.0;

  // Output compact JSON — one line per sample
  Serial.print("{\"r\":");
  Serial.print(roll, 2);
  Serial.print(",\"p\":");
  Serial.print(pitch, 2);
  Serial.print(",\"y\":");
  Serial.print(yaw, 2);
  Serial.print(",\"ax\":");
  Serial.print(ax_g, 4);
  Serial.print(",\"ay\":");
  Serial.print(ay_g, 4);
  Serial.print(",\"az\":");
  Serial.print(az_g, 4);
  Serial.print(",\"t\":");
  Serial.print(now);
  Serial.println("}");
}
