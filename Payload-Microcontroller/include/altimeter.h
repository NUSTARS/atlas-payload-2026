// altimeter.h
#pragma once 

#include <Arduino.h>
#include <Adafruit_BMP280.h>

struct AltimeterData {
    float temp_C;
    float pressure_hPa;
    float altitude_m;
};

//
// init altimeter: (void) -> bool
//
// initializes the altimeter
// returns true in the event of success, false otherwise
bool initAltimeter();


// read altimeter: (void) -> float
// reads the current altitude from the altimeter
void readAltimeter(AltimeterData &data);