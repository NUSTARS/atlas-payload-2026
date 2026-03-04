#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO_UART   Serial2 // Teensy 4.1 RX2/TX2 (pins 7/8)
static const uint32_t BNO_BAUD  = 115200;  // common for BNO08x UART
static const uint32_t REPORT_US = 5000;     // 200 Hz

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
// Setup BNO08x over UART
// =====================
bool initIMU() {
  BNO_UART.begin(BNO_BAUD);

  Serial.println("Searching for BNO085");

  // If your Adafruit_BNO08x version doesn't have begin_UART(), paste the error
  if (!bno.begin_UART(&BNO_UART)) {
    Serial.println("ERROR: BNO08x not detected over UART");
    Serial.println("Check RX2/TX2 wiring, baud, and BNO08x UART mode");
    return false;
  }

  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER,        REPORT_US);
  ok &= bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_US);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR,      REPORT_US);
  return ok;
}


// =====================
// Update IMUData directly from BNO events
// =====================
// Notes:
// - BNO gives accel, gyro, and quat (rotation vector). Your struct uses YPR,
//   so we leave ypr[] as-is unless you add quat->ypr conversion.
// - timeUTC here uses millis() as a stand-in. Replace with GPS/RTC if you have it.
// - posLla / velBody / insStatus are not provided by BNO alone; we keep them as-is
//   so another module can set them without us clobbering them every update.
void readIMU(IMUData &data) {
  data.timeUTC = 0.001 * (double)millis();

  // Drain at most N events per call (prevents lockups if backlog grows)
  for (int n = 0; n < 20; n++) {
    if (!bno.getSensorEvent(&val)) break;

    switch (val.sensorId) {
      case SH2_ACCELEROMETER:
        data.accel[0] = val.un.accelerometer.x;
        data.accel[1] = val.un.accelerometer.y;
        data.accel[2] = val.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        data.angularRate[0] = val.un.gyroscope.x;
        data.angularRate[1] = val.un.gyroscope.y;
        data.angularRate[2] = val.un.gyroscope.z;
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
        if (fabsf(sinp) >= 1.0f) data.ypr[1] = copysignf(1.57079632679f, sinp);
        else                    data.ypr[1] = asinf(sinp);

        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        data.ypr[2] = atan2f(sinr_cosp, cosr_cosp);
        break;
      }
    }
  }
}