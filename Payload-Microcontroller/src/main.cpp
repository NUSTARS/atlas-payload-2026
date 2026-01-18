// main.cpp

// INCLUDES
#include <Arduino.h>

#include <Eigen.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
#include <dsp.h>

// GLOBALS



// INTERRUPTS


// main
void setup() {
  Serial.begin(115200); // Initialize serial communication
}

void loop() {
  Serial.println("Hello, World!"); // Print "Hello, World!" to the Serial Monitor
  delay(1000); // Wait for 1 second
}