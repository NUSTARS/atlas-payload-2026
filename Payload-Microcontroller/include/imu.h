// imu.h
#pragma once

#include <Arduino.h>

struct IMUData {
    uint64_t timeUTC; // in ms
    float ypr[3];
    float angularRate[3];
    float accel[3];
    double posLla[3];
    // float velBody[3];
    // uint16_t insStatus;
};

enum IMUReadStatus {
    IMU_READ_OK = 0,
    IMU_READ_TIMEOUT,
    IMU_READ_CRC_FAIL
};

// Initialize Serial2 for the IMU at the given baud rate and send configuration commands.
bool initIMU(uint32_t baudRate = 115200);

// Read one full IMU frame with timeout. On success updates the latest IMU sample.
IMUReadStatus readIMUFrameBlocking(uint32_t timeoutMs);

// Copy the most recently published complete IMU packet into out.
// Returns true if at least one valid packet has been received, false otherwise.
// Safe to call from the 50Hz control ISR.
bool getLatestIMUData(IMUData &out);

// Monotonic counter incremented each time a valid IMU frame is published.
// Use this to run control exactly once per new IMU sample.
uint32_t getIMUSequence();

// Atomically read and clear the frame-ready flag.
// Returns true if a new frame has arrived since last call, false otherwise.
// Safe to call from ISR context.
bool clearIMUFrameReady();

