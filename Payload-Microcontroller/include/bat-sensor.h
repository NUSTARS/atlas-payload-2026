#pragma once
#include <INA226.h>
#include <Wire.h>

struct BatteryData {
    float voltage_v;
    float current_ma;
    float power_mw;
    float load_voltage_V;
};

// initBatSensor: (void) -> (bool)
// initializes the battery sensor
bool initBatSensor();

// readBatSensor: (void) -> (void)
// returns the current battery sensor reading
void readBatSensor(BatteryData& data);