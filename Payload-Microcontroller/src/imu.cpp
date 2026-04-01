// imu.cpp
//
// Serial IMU interface for VectorNav binary protocol.
// serviceIMU() drains Serial2 each loop iteration; getLatestIMUData() is safe
// to call from the 50Hz control ISR (double-buffer, minimal critical section).

#include "imu.h"
#include <Arduino.h>
#include <string.h>

// Internal constants

static const size_t FRAME_LEN = 88; // fixed binary packet length

// Internal buffers

// Ring buffer: no memmove, just head/tail pointers.
static const size_t RX_BUF_SIZE = FRAME_LEN * 4;
static uint8_t rx_buf[RX_BUF_SIZE];
static size_t rx_head = 0;  // write pointer
static size_t rx_tail = 0;  // read pointer

// Loop-side parse target: written by parseFrame()
static IMUData imu_staging;

// ISR-readable snapshot: updated atomically by publishFrame().
static IMUData imu_published;
static volatile bool imu_valid = false;
static volatile uint32_t imu_seq = 0;

// checksum
static uint16_t crc16_step(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int b = 0; b < 8; b++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// Frame parser
// Parses a validated VN binary frame into imu_staging, then
// atomically copies to imu_published with interrupts disabled.
static void parseFrame(const uint8_t *frame) {
    // frame[0] == 0xFA (already verified by serviceIMU)
    uint8_t groupmask = frame[1];

    uint16_t masks[8] = {0};
    int maskIdx = 0;
    for (int i = 0; i < 8; i++) {
        if (groupmask & (1 << i)) {
            memcpy(&masks[i], &frame[2 + maskIdx * 2], 2);
            maskIdx++;
        }
    }

    const uint8_t *ptr = &frame[2 + maskIdx * 2];

    // Group 1 (Common)
    if (masks[0] & (1 << 3)) { memcpy(imu_staging.ypr,          ptr, 12); ptr += 12; }
    if (masks[0] & (1 << 5)) { memcpy(imu_staging.angularRate,  ptr, 12); ptr += 12; }
    if (masks[0] & (1 << 8)) { memcpy(imu_staging.accel,        ptr, 12); ptr += 12; }

    // Group 2 (Time)
    if (masks[1] & (1 << 6)) { memcpy(&imu_staging.timeUTC,     ptr,  8); ptr +=  8; }

    // Group 6 (INS)
    if (masks[5] & (1 << 0)) { memcpy(&imu_staging.insStatus,   ptr,  2); ptr +=  2; }
    if (masks[5] & (1 << 1)) { memcpy(imu_staging.posLla,       ptr, 24); ptr += 24; }
    if (masks[5] & (1 << 3)) { memcpy(imu_staging.velBody,      ptr, 12); ptr += 12; }

    // send full frame to published buffer
    noInterrupts();
    memcpy(&imu_published, &imu_staging, sizeof(IMUData));
    imu_valid = true;
    imu_seq++;
    interrupts();
}

bool initIMU(uint32_t baudRate) {
    Serial2.begin(baudRate);
    delay(100); // let the port settle

    // // Disable asynchronous ASCII output so only binary packets arrive
    // Serial2.println("$VNASY,0*4E");
    // delay(50);
    // // Set binary output register (Group 1: YPR + AngRate + Accel; Group 2: TimeUTC; Group 6: INS)
    // Serial2.println("$VNWRG,06,17,0*XX");
    // delay(50);

    return true;
}

static size_t ringAvail() {
    if (rx_head >= rx_tail) return rx_head - rx_tail;
    return RX_BUF_SIZE - rx_tail + rx_head;
}

static uint8_t ringPeek(size_t offset) {
    return rx_buf[(rx_tail + offset) % RX_BUF_SIZE];
}

static void ringConsume(size_t n) {
    rx_tail = (rx_tail + n) % RX_BUF_SIZE;
}

// serviceIMU() — call from main loop every iteration.
// Drains Serial2 into ring buffer, then processes as many complete frames as possible.
// On each valid frame: parse and atomically publish to imu_published.
// On sync loss or CRC failure: discard one byte and resync.
void serviceIMU() {
    // 1. Drain incoming bytes (non-blocking)
    while (Serial2.available()) {
        size_t nextHead = (rx_head + 1) % RX_BUF_SIZE;
        if (nextHead == rx_tail) break;  // buffer full, drop byte
        rx_buf[rx_head] = (uint8_t)Serial2.read();
        rx_head = nextHead;
    }

    // 2. Process complete frames
    while (ringAvail() >= FRAME_LEN) {
        // Require sync byte at front
        if (ringPeek(0) != 0xFA) {
            // Resync: advance one byte
            ringConsume(1);
            continue;
        }

        // Validate CRC
        uint16_t computed = 0;
        for (size_t i = 1; i < FRAME_LEN - 2; i++) {
            computed = crc16_step(computed, ringPeek(i));
        }

        uint16_t received = ((uint16_t)ringPeek(FRAME_LEN - 2) << 8) | (uint16_t)ringPeek(FRAME_LEN - 1);

        if (computed != received) {
            // Bad CRC: discard this sync byte and resync
            ringConsume(1);
            continue;
        }

        // Valid frame: copy to staging, parse, and publish
        uint8_t frame[FRAME_LEN];
        for (size_t i = 0; i < FRAME_LEN; i++) {
            frame[i] = ringPeek(i);
        }
        parseFrame(frame);
        ringConsume(FRAME_LEN);
    }
}

// getLatestIMUData() — safe to call from the 50Hz control ISR.
// Returns false (and leaves out unchanged) if no valid packet has arrived yet.
bool getLatestIMUData(IMUData &out) {
    if (!imu_valid) return false;
    memcpy(&out, &imu_published, sizeof(IMUData));
    // Serial.println("got imu data, it was ");
    // Serial.println(imu_published.timeUTC, 6);
    // Serial.println("");
    return true;
}

uint32_t getIMUSequence() {
    return imu_seq;
}
