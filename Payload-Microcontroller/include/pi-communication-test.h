#pragma once
#include <Arduino.h>

extern volatile uint32_t spi_events;
void setup_slave();
void spi_set_imu_packet(const float accel[3], const float gyro[3], const float quat[4]);
