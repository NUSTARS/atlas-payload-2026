#include "Sensor.hpp"

// C:\Users\prest\Documents\GitHub\NUSTARS\atlas-payload-2026\Roll-Control\VN200 Test\examples\TeensyTest\include\vectornav\HAL\Serial.hpp
// C:\Users\prest\Documents\GitHub\NUSTARS\atlas-payload-2026\Roll-Control\VN200 Test\examples\TeensyTest\vectornav\Config.hpp



VN::Sensor sensor;

void setup() {
    // USB Serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}  // Wait for Serial Monitor
    
    Serial.println("=== VectorNav Teensy Loopback Test ===");
    Serial.println("Connect Pin 1 to Pin 0 with a jumper wire");
    delay(1000);
    
    // Connect to "Serial1" (which will loopback via wire)
    VN::Error err = sensor.connect("Serial1", 115200);
    
    if (err != VN::Error::None) {
        Serial.print("Connection failed! Error code: ");
        Serial.println(static_cast<int>(err));
        while(1) { delay(1000); }
    }
    
    Serial.println("Port opened successfully!");
    
    // TODO: Add your loopback test code here
}

void loop() {
    // Your code here
}