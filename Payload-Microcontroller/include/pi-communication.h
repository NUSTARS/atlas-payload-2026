// pi-communication.h
#pragma once
#include <Arduino.h>
#include <string.h>

#define CS_PIN 10

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

struct __attribute__((packed)) Packet { // 49 bytes + 8 bytes for header
    float orientation[3]; // 12 bytes
    float angular_velocity[3]; // 12 bytes
    double gps_long; // 8 bytes
    double gps_lat;  // 8 bytes
    uint8_t state;  // 1 byte
    float altitude_m; // 4 bytes
    float battery_v; // 4 bytes
};
static_assert(sizeof(Packet)==49, "...");

void setup_slave();

// builds spi packet based on existing spi packet str
void build_spi_buffer();

// sets the voltage field of the spi packet struct
void spi_packet_set_voltage(const float voltage);

// sets the altitude field of the spi packet struct
void spi_packet_set_altitude(const float altitude);

// sets the state field fo the spi packet struct
void spi_packet_set_state(const uint8_t state);

// sets the orientation and angular velocity fields of the packet struct
void spi_packet_set_imu(const float (&orientation)[3], const float (&angular_velocity)[3], const double (&latLonAlt)[3]);

