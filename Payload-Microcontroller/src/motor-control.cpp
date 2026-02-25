// How to read error from motor using UART:

/*
1. Output is number
2. Convert number to binary
3. Bit field determines which error

Example:
1. Output is 64
2. Binary is 0b1000000
3. Bit field is 6 (starts at 0, from LSB)

Common errors:
0	INVALID_STATE (0)
1	MOTOR_FAILED (1)
6	WATCHDOG_TIMER_EXPIRED (64)
8	ESTOP_REQUESTED (256)
*/
#include <Arduino.h>


String readODrive(String cmd) {
  // Clear the buffer of any old junk
  while(Serial1.available()) Serial1.read(); 

  Serial1.println(cmd); // Sends command + \n

  // Teensy 4.1 is fast; give the ODrive a moment to think
  Serial1.setTimeout(50); 
  String resp = Serial1.readStringUntil('\n');
  
  resp.trim(); // Remove any stray \r or whitespace
  return resp;
}

void setup() {
  Serial.begin(115200);
  // ODrive S1 default baud is 115200, but ensure your 
  // ODrive tool settings match this!
  Serial1.begin(115200); 
  
  while (!Serial && millis() < 2000); // Wait for PC monitor
  Serial.println("ODrive S1 Error Reader Initialized");
}

void loop() {
  // On ODrive S1, 'active_errors' is the common check
  String axisErr = readODrive("r axis0.active_errors");

  if (axisErr.length() > 0) {
    Serial.print("Axis error (Hex): 0x");
    Serial.println(axisErr);
  } else {
    Serial.println("Timeout: No response from ODrive");
  }

  delay(500);
}