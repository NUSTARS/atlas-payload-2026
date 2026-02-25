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
    float orientation[3]; // 12 bytes
    float angular_velocity[3]; // 12 bytes
    float gps_long; // 4 bytes
    float gps_lat;  // 4 bytes
    uint8_t state;  // 1 byte
    float altitude_m; // 4 bytes
    float battery_v; // 4 bytes
};

void setup_slave();

// builds spi packet based on existing spi packet str
void build_spi_buffer();

void spi_set_packet(const float accel[3], const float gyro[3], const float quat[4]);
// sets the voltage field of the spi packet struct
void spi_packet_set_voltage(const float voltage);
// sets the altitude field of the spi packet struct
void spi_packet_set_altitude(const float altitude);
// sets the state field fo the spi packet struct
void spi_packet_set_state(const uint8_t state);
// sets the orientation and angular velocity fields of the packet struct
void spi_packet_set_imu_data(const float (&orientation)[3], const float (&angular_velocity)[3]);

