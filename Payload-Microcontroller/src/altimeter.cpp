#include "altimeter.h"

#define BMP5XX_CS_PIN 10

Adafruit_BMP5xx bmp;

// Sea level pressure in hPa
float groundPressureHPa = 1013.25;

// initAltimeter: (void) -> (void)
// initializes the altimeter
bool initAltimeter() {
	if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
		Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
		return false;
	}
	Serial.println(F("BMP5xx found!"));	
	return true;
}

// readAltimeter: (void) -> float
// reads the current altitude from the altimeter
void readAltimeter(AltimeterData &data) {
	// Data is ready, perform reading
	if (!bmp.performReading()) {
		return;
	}

	data.temp_C = bmp.temperature;
	data.pressure_hPa = bmp.pressure; 
	data.altitude_m = bmp.readAltitude(groundPressureHPa);
}
//If you want the altitude to start at 0.0 at the launch site, you should capture the ground pressure during setup()