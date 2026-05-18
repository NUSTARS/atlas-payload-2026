// imu.cpp
//
// Serial IMU interface for VectorNav binary protocol.
// Reads one full frame in a blocking call from Serial5.

#include "imu.h"
#include <Arduino.h>
#include <string.h>

static const size_t FRAME_LEN = 50; // fixed binary packet length
static IMUData imu_latest;
static bool imu_valid = false;

// checksum
static uint16_t crc16_step(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int b = 0; b < 8; b++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
static inline uint64_t read_u64_raw(const uint8_t *p) {
    return ((uint64_t) p[0] |
            (uint64_t) p[1] << 8 |
            (uint64_t) p[2] << 16 | 
            (uint64_t) p[3] << 24 | 
            (uint64_t) p[4] << 32 |
            (uint64_t) p[5] << 40 | 
            (uint64_t) p[6] << 48 | 
            (uint64_t) p[7] << 56);
}

static inline uint32_t read_u32_raw(const uint8_t *p) {
    return ((uint32_t) p[0] | 
            (uint32_t) p[1] << 8 | 
            (uint32_t) p[2] << 16 | 
            (uint32_t) p[3] << 24);
}

static inline float read_f32(const uint8_t *p) {
    uint32_t raw = read_u32_raw(p);
    float val;
    memcpy(&val, &raw, sizeof(val));
    return val;
}

static inline double read_f64(const uint8_t *p) {
    uint64_t raw = read_u64_raw(p);
    double val;
    memcpy(&val, &raw, sizeof(val));
    return val;
}

static void parseFrame(const uint8_t *frame) {
    if (frame[0] != 0xFA) { // invalid
        return;
    }
    imu_latest.timeUTC = read_u64_raw(&frame[4]) * 1e-6;
    imu_latest.ypr[0] = read_f32(&frame[12]);
    imu_latest.ypr[1] = read_f32(&frame[16]);
    imu_latest.ypr[2] = read_f32(&frame[20]);
    imu_latest.angularRate[0] = read_f32(&frame[24]);
    imu_latest.angularRate[1] = read_f32(&frame[28]);
    imu_latest.angularRate[2] = read_f32(&frame[32]);
    // imu_latest.posLla[0] = read_f64(&frame[36]);
    // imu_latest.posLla[1] = read_f64(&frame[44]);
    // imu_latest.posLla[2] = read_f64(&frame[52]);
    // imu_latest.velBody[0] = read_f32(&frame[60]);
    // imu_latest.velBody[1] = read_f32(&frame[64]);
    // imu_latest.velBody[2] = read_f32(&frame[68]);
    imu_latest.accel[0] = read_f32(&frame[36]);
    imu_latest.accel[1] = read_f32(&frame[40]);
    imu_latest.accel[2] = read_f32(&frame[44]);
    // imu_latest.insStatus = ((uint16_t) frame[84] | ((uint16_t) frame[85] << 8));

    imu_valid = true;
}

bool initIMU(uint32_t baudRate) {
    Serial5.begin(baudRate);
    delay(100); // let the port settle

    // // Disable asynchronous ASCII output so only binary packets arrive
    // Serial5.println("$VNASY,0*4E");
    // delay(50);
    // // Set binary output register (Group 1: YPR + AngRate + Accel; Group 2: TimeUTC; Group 6: INS)
    // Serial5.println("$VNWRG,06,17,0*XX");
    // delay(50);

    return true;
}

IMUReadStatus readIMUFrameBlocking(uint32_t timeoutMs) {
    const uint32_t timeoutUs = timeoutMs * 1000UL;
    const uint32_t startUs = micros();
    uint8_t frame[FRAME_LEN];
    size_t idx = 0;
    bool sawCrcFail = false;

    while ((uint32_t)(micros() - startUs) < timeoutUs) {
        if (!Serial5.available()) {
            continue;
        }

        uint8_t byte = (uint8_t)Serial5.read();
        if (idx == 0) {
            if (byte != 0xFA) {
                continue;
            }
            frame[idx++] = byte;
            continue;
        }

        frame[idx++] = byte;
        if (idx < FRAME_LEN) {
            continue;
        }

        uint16_t computed = 0;
        for (size_t i = 1; i < FRAME_LEN - 2; i++) {
            computed = crc16_step(computed, frame[i]);
        }

        uint16_t received = ((uint16_t)frame[FRAME_LEN - 2] << 8) | (uint16_t)frame[FRAME_LEN - 1];
        if (computed != received) {
            sawCrcFail = true;

            // Resynchronize by finding the next possible sync byte in the current buffer.
            size_t nextSync = FRAME_LEN;
            for (size_t i = 1; i < FRAME_LEN; i++) {
                if (frame[i] == 0xFA) {
                    nextSync = i;
                    break;
                }
            }

            if (nextSync < FRAME_LEN) {
                size_t remaining = FRAME_LEN - nextSync;
                memmove(frame, &frame[nextSync], remaining);
                idx = remaining;
            } else {
                idx = 0;
            }
            continue;
        }

        parseFrame(frame);
        return IMU_READ_OK;
    }

    return sawCrcFail ? IMU_READ_CRC_FAIL : IMU_READ_TIMEOUT;
}

bool getLatestIMUData(IMUData &out) {
    if (!imu_valid) {
        return false;
    }
    memcpy(&out, &imu_latest, sizeof(IMUData));
    return true;
}
