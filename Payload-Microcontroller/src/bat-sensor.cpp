// bat-sensor.cpp
//
//

#include "bat-sensor.h"
// initBatSensor: (void) -> (void)
// initializes the battery sensor

Adafruit_INA219 ina219;

bool initBatSensor() {
	if (!ina219.begin()){
		Serial.println("Failed to connect to sensor (Battery)");
		return false;
	}

	Serial.println("Battery sensor intialized successfully");
	return true;
}

// readBatSensor: (void) -> (void)
// returns the current battery sensor reading
void readBatSensor(BatteryData &data) {
	data.voltage_v = ina219.getBusVoltage_V();
	data.current_ma = ina219.getCurrent_mA();
	data.power_mw = ina219.getPower_mW();


	float shunt_voltage_mV = ina219.getShuntVoltage_mV();
	data.load_voltage_V = data.voltage_v + (shunt_voltage_mV / 1000.0);
}