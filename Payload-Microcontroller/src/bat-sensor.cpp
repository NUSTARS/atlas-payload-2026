#include "bat-sensor.h"

INA226 INA(0x40); // or whatever i2c address it's at

// initBatSensor: (void) -> (bool)
// initializes the battery sensor
bool initBatSensor() {
	Wire.begin();
	if (!INA.begin()) {
		Serial.println("could not connect. Fix and Reboot");
		return false;
	}
	INA.setMaxCurrentShunt(5.0, 0.01);
	Serial.println("Found INA!");
	return true;
}

// readBatSensor: (void) -> (void)
// returns the current battery sensor reading
void readBatSensor(BatteryData &data) {
	data.voltage_v = INA.getBusVoltage();
	data.current_ma = INA.getCurrent_mA();
	data.power_mw = INA.getPower_mW();
	float shunt_voltage_mV = INA.getShuntVoltage_mV();
	data.load_voltage_V = data.voltage_v + (shunt_voltage_mV / 1000.0);

	float vsh_mV = INA.getShuntVoltage_mV();
	Serial.print("Shunt (mV): ");
	Serial.println(vsh_mV, 3);

	Serial.print("Bus (V): ");
	Serial.println(INA.getBusVoltage(), 3);
}