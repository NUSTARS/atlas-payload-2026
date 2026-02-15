// pi-communication.h
#pragma once
#include <Arduino.h>
#include <string.h>

// 
// Packet Message:
//      - Magic Bytes (8 bytes)
//      - Seq Number (8 bytes)
//      - Orientation (12 bytes)
//      - Velocity (12 bytes)
//      - GPS Long, Lat (8 bytes)
//      - State (1 byte)
//      - Altimeter (4 bytes)
//      - Voltage (1 byte) 
//      - Checksum (4 bytes)
//
//      Total: 58 bytes + 6 bytes -> padding 64 bytes

struct Packet {
    
}

void setup_slave();
void spi_set_imu_packet(const float accel[3], const float gyro[3], const float quat[4]);