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

// Initialize Serial2 for the IMU at the given baud rate and send configuration commands.
bool initIMU(uint32_t baudRate = 115200);

// Service the serial RX buffer. Call every main loop iteration.
// Drains Serial2 into an internal buffer, assembles complete 88-byte packets,
// validates CRC-16, and atomically publishes the latest valid packet.
void serviceIMU();

// Copy the most recently published complete IMU packet into out.
// Returns true if at least one valid packet has been received, false otherwise.
// Safe to call from the 50Hz control ISR.
bool getLatestIMUData(IMUData &out);

// Monotonic counter incremented each time a valid IMU frame is published.
// Use this to run control exactly once per new IMU sample.
uint32_t getIMUSequence();


