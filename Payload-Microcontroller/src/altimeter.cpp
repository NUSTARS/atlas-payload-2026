// #include "altimeter.h"

// #define BMP5XX_CS_PIN 6

// // Adafruit_BMP5xx bmp;

// // Sea level pressure in hPa
// float groundPressureHPa = 1013.25;

// // initAltimeter: (void) -> (void)
// // initializes the altimeter
// bool initAltimeter() {
// 	pinMode(BMP5XX_CS_PIN, OUTPUT);
// 	digitalWrite(BMP5XX_CS_PIN, HIGH);  

// 	SPI1.setMISO(1);
// 	SPI1.setMOSI(26);
// 	SPI1.setSCK(27);
// 	SPI1.begin();

//     Serial.println("Searching for Altimeter...");
// 	if (!bmp.begin(BMP5XX_CS_PIN, &SPI1)) {
// 		Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
// 		return false;
// 	}
// 	Serial.println(F("BMP5xx found!"));	
// 	return true;
// }

// // readAltimeter: (void) -> float
// // reads the current altitude from the altimeter
// void readAltimeter(AltimeterData &data) {
// 	// Data is ready, perform reading
// 	if (!bmp.performReading()) {
// 		return;
// 	}

// 	data.temp_C = bmp.temperature;
// 	data.pressure_hPa = bmp.pressure; 
// 	data.altitude_m = bmp.readAltitude(groundPressureHPa);
// }
// //If you want the altitude to start at 0.0 at the launch site, you should capture the ground pressure during setup()


#include "altimeter.h"
#include <Wire.h>

Adafruit_BMP5xx bmp;

// Sea level pressure in hPa
float groundPressureHPa = 1013.25;

// initializes the altimeter over I2C
bool initAltimeter() {
    Wire.begin();

    // Default BMP58x I2C address is usually 0x47 or 0x46 depending on SDO
    if (!bmp.begin(0x47, &Wire)) {
        Serial.println(F("Could not find a valid BMP5xx sensor over I2C, check wiring!"));
        return false;
    }

    Serial.println(F("BMP5xx found over I2C!"));
    return true;
}

// reads the current altitude from the altimeter
void readAltimeter(AltimeterData &data) {
    if (!bmp.performReading()) {
        return;
    }

    data.temp_C = bmp.temperature;
    data.pressure_hPa = bmp.pressure / 100.0f;
    data.altitude_m = bmp.readAltitude(groundPressureHPa);
}