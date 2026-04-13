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
static volatile bool imu_frame_ready = false;

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

// Frame parser
// Parses a validated VN binary frame into imu_staging, then
// atomically copies to imu_published with interrupts disabled.
static void parseFrame(const uint8_t *frame) {
    if (frame[0] != 0xFA) { // invalid
        // throw std::runtime_error("invalid message");

    }
    // Serial.println("parsing frame");
    // // print frame for debugging
    // for (size_t i = 0; i < FRAME_LEN; i++) {
    //     Serial.print(frame[i], HEX); Serial.print(" ");
    // }
    imu_staging.timeUTC = read_u64_raw(&frame[4]) * 1e-6;
    imu_staging.ypr[0] = read_f32(&frame[12]);
    imu_staging.ypr[1] = read_f32(&frame[16]);
    imu_staging.ypr[2] = read_f32(&frame[20]);
    imu_staging.angularRate[0] = read_f32(&frame[24]);
    imu_staging.angularRate[1] = read_f32(&frame[28]);
    imu_staging.angularRate[2] = read_f32(&frame[32]);
    imu_staging.posLla[0] = read_f64(&frame[36]);
    imu_staging.posLla[1] = read_f64(&frame[44]);
    imu_staging.posLla[2] = read_f64(&frame[52]);
    imu_staging.velBody[0] = read_f32(&frame[60]);
    imu_staging.velBody[1] = read_f32(&frame[64]);
    imu_staging.velBody[2] = read_f32(&frame[68]);
    imu_staging.accel[0] = read_f32(&frame[72]);
    imu_staging.accel[1] = read_f32(&frame[76]);
    imu_staging.accel[2] = read_f32(&frame[80]);
    imu_staging.insStatus = ((uint16_t) frame[84] | ((uint16_t) frame[85] << 8));

    // Atomically publish to the ISR-readable snapshot
    noInterrupts();
    imu_published = imu_staging;
    imu_valid = true;
    imu_seq++;
    imu_frame_ready = true;
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

// clearIMUFrameReady() — safe to call from ISR.
// Atomically reads and clears the imu_frame_ready flag.
// Returns true if a frame has arrived since last call, false otherwise.
bool clearIMUFrameReady() {
    noInterrupts();
    bool ready = imu_frame_ready;
    imu_frame_ready = false;
    interrupts();
    return ready;
}
