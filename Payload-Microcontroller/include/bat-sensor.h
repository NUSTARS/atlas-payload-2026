// bat-sensor.h
#pragma once
#include <Arduino.h>
#include <Adafruit_INA219.h>


struct BatteryData {
    float voltage_v;
    float current_ma;
    float power_mw;
    float load_voltage_V;
};
// initBatSensor: (void) -> (void)
// initializes the battery sensor
bool initBatSensor();

// readBatSensor: (void) -> (void)
// returns the current battery sensor reading
void readBatSensor(BatteryData &data);