// imu.h
#pragma once

#include <Arduino.h>

struct IMUData {
    uint64_t timeUTC; // in ms
    float ypr[3];
    float angularRate[3];
    float accel[3];
    double posLla[3];
    float velBody[3];
    uint16_t insStatus;
};

enum IMUReadStatus {
    IMU_READ_OK = 0,
    IMU_READ_TIMEOUT,
    IMU_READ_CRC_FAIL
};

// Initialize Serial5 for the IMU at the given baud rate.
bool initIMU(uint32_t baudRate = 115200);

// Blocking receive for one full binary frame.
// Returns detailed status for timeout/CRC debugging.
IMUReadStatus readIMUFrameBlocking(uint32_t timeoutMs);

// Copy the most recently parsed IMU packet into out.
// Returns true if at least one valid packet has been received.
bool getLatestIMUData(IMUData &out);


