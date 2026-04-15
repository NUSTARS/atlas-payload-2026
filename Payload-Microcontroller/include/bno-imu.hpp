#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

static const uint32_t REPORT_US = 5000;   // 200 Hz

static const uint32_t CONTROL_PERIOD_us = 500000; // 500 ms


struct IMUData {
  double timeUTC;
  float ypr[3];
  float angularRate[3];
  float accel[3];
  double posLla[3];
  float velBody[3];
  uint16_t insStatus;
};

Adafruit_BNO08x bno;
sh2_SensorValue_t val;

// =====================
// Setup BNO08x over I2C (Wire1)
// =====================
bool initIMU() {
  Wire1.begin();          // use SDA1 / SCL1 at default 100 kHz for init

  // Serial.println("Before begin_I2C (Wire1)");
  bool ok = bno.begin_I2C(0x4A, &Wire1);  // explicitly pass Wire1
  // Serial.println("After begin_I2C");

  // Raise to 400 kHz only after successful handshake — setting it before
  // begin_I2C causes init failure because the chip isn't ready for fast-mode yet
  if (ok) Wire1.setClock(400000);

  if (!ok) {
    Serial.println("ERROR: BNO08x not detected over I2C (Wire1)");
    Serial.println("Check SDA1/SCL1 wiring, power, and address");
    return false;
  }

  Serial.println("BNO08x Found!");

  if (!bno.enableReport(SH2_ACCELEROMETER, REPORT_US)) {
    Serial.println("Failed to enable accelerometer");
    return false;
  }

  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_US)) {
    Serial.println("Failed to enable gyroscope");
    return false;
  }

  if (!bno.enableReport(SH2_ROTATION_VECTOR, REPORT_US)) {
    Serial.println("Failed to enable rotation vector");
    return false;
  }

  return true;
}

// Bitmask flags for which sensor fields were updated in a readIMU() call
static constexpr uint8_t IMU_GOT_ACCEL  = 0x01;
static constexpr uint8_t IMU_GOT_GYRO   = 0x02;
static constexpr uint8_t IMU_GOT_ROTVEC = 0x04;
static constexpr uint8_t IMU_GOT_ALL    = IMU_GOT_ACCEL | IMU_GOT_GYRO | IMU_GOT_ROTVEC;

// =====================
// Update IMUData directly from BNO events.
// Returns a bitmask of which fields were actually updated this call (IMU_GOT_*).
// Fields not updated retain their previous values — callers should check the
// bitmask before trusting data, to avoid using stale carried-over values.
// =====================
uint8_t readIMU(IMUData &data) {
  data.timeUTC = 0.001 * (double)millis();
  uint8_t got = 0;

  for (int n = 0; n < 20; n++) {
    if (!bno.getSensorEvent(&val)) break;

    switch (val.sensorId) {
      case SH2_ACCELEROMETER:
        data.accel[0] = val.un.accelerometer.x;
        data.accel[1] = val.un.accelerometer.y;
        data.accel[2] = val.un.accelerometer.z;
        got |= IMU_GOT_ACCEL;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        data.angularRate[0] = val.un.gyroscope.x;
        data.angularRate[1] = val.un.gyroscope.y;
        data.angularRate[2] = val.un.gyroscope.z;
        got |= IMU_GOT_GYRO;
        break;

      case SH2_ROTATION_VECTOR: {
        float w = val.un.rotationVector.real;
        float x = val.un.rotationVector.i;
        float y = val.un.rotationVector.j;
        float z = val.un.rotationVector.k;

        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        data.ypr[0] = atan2f(siny_cosp, cosy_cosp);

        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
          data.ypr[1] = copysignf(1.57079632679f, sinp);
        } else {
          data.ypr[1] = asinf(sinp);
        }

        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        data.ypr[2] = atan2f(sinr_cosp, cosr_cosp);
        got |= IMU_GOT_ROTVEC;
        break;
      }
    }
  }

  return got;
}