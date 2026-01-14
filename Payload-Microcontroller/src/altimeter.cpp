// altimeter.cpp
//
//
#include "altimeter.h"

Adafruit_BMP280 bmp;
// Sea level pressure in hPa
float groundPressureHPa = 1013.25;
// initAltimeter: (void) -> (void)
// initializes the altimeter
bool initAtimeter() {
	if (!bmp.begin(0x47)){
		Serial.println("Failed to connect sensor (Altimeter)");
		return false;
	}
	// High-performance settings for rocket/drone flight
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);  /* Standby time. */

	

	delay(100);
    float totalPressure = 0;
	int samples = 20;
    for(int i = 0; i < samples; i++) {
        totalPressure += (bmp.readPressure() / 100.0);
        delay(10);
    }
    groundPressureHPa = totalPressure / 10.0;

	Serial.println("Altimeter initialized successfully");
	return true;
}

// readAltimeter: (void) -> float
// reads the current altitude from the altimeter
void readAltimeter(AltimeterData &data) {

	data.temp_C = bmp.readTemperature();
	data.pressure_hPa = bmp.readPressure() / 100.0; // Convert Pa to hPa
	// Hypsometric formula to calculate altitude from pressure
	data.altitude_m = 44330.0 * (1.0 - pow(data.pressure_hPa / groundPressureHPa, 0.1903));
}
//If you want the altitude to start at 0.0 at the launch site, you should capture the ground pressure during setup()