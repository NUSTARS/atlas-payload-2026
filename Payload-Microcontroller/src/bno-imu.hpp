float accel[3] = {0, 0, 0};   // x, y, z
float gyro[3]  = {0, 0, 0};   // x, y, z
float quat[4]  = {0, 0, 0, 0}; // w, x, y, z (real, i, j, k)

Adafruit_BNO08x bno;
sh2_SensorValue_t val;

// report interval in microseconds
static const uint32_t REPORT_US = 5000; // 200 Hz

bool setup_bno() {

    if (!bno.begin_I2C(0x4A, &Wire)) {  // 0x4A is default Adafruit addr
        Serial.println("ERROR: BNO085 not detected over I2C");
        Serial.println("Check SDA/SCL wiring and power");
        return false;
    }

    Serial.println("BNO085 detected!");
    
    bool success = true;
    success &= bno.enableReport(SH2_ACCELEROMETER, REPORT_US);
    success &= bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_US);
    success &= bno.enableReport(SH2_ROTATION_VECTOR, REPORT_US);

    return success;
} 

void update_bno_values() {
    if (bno.getSensorEvent(&val)) {
        switch (val.sensorId) {
            case SH2_ACCELEROMETER:
                accel[0] = val.un.accelerometer.x;
                accel[1] = val.un.accelerometer.y;
                accel[2] = val.un.accelerometer.z;
                break;

            case SH2_GYROSCOPE_CALIBRATED:
                gyro[0] = val.un.gyroscope.x;
                gyro[1] = val.un.gyroscope.y;
                gyro[2] = val.un.gyroscope.z;
                break;

            case SH2_ROTATION_VECTOR:
                quat[0] = val.un.rotationVector.real;
                quat[1] = val.un.rotationVector.i;
                quat[2] = val.un.rotationVector.j;
                quat[3] = val.un.rotationVector.k;
                break;
        }
    }
}

float* get_acceleration() {
    return accel;
}

float* get_gyro() {
    return gyro;
}

float* get_quat() {
    return quat;
}

void printXYZ(const char* label, float v[3]) {
    Serial.print(label);
    Serial.print(": ");
    Serial.print(v[0], 4); Serial.print(", ");
    Serial.print(v[1], 4); Serial.print(", ");
    Serial.println(v[2], 4);
}

void printQuat(const char* label, float q[4]) {
    Serial.print(label);
    Serial.print(": ");
    Serial.print(q[0], 4); Serial.print(", ");
    Serial.print(q[1], 4); Serial.print(", ");
    Serial.print(q[2], 4); Serial.print(", ");
    Serial.println(q[3], 4);
}
