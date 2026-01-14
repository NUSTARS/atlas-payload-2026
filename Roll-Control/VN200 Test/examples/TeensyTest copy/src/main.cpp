#include <Arduino.h>
#include "vectornav/Sensor.hpp"

VN::Sensor sensor;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== VectorNav Teensy Test ===");
    Serial.println("Connecting to Serial1...");
    
    VN::Error err = sensor.connect("Serial1", 115200);
    
    if (err == VN::Error::None) {
        Serial.println("✓ Connected successfully!");
    } else {
        Serial.print("✗ Connection failed - Error: ");
        Serial.println(static_cast<int>(err));
    }
}

void loop() {
    // Your code here
}