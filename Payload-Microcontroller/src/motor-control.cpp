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

 
// #include <ODriveUART.h>
// #include <SoftwareSerial.h>

// // Documentation for this example can be found here:
// // https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html


// ////////////////////////////////
// // Set up serial pins to the ODrive
// ////////////////////////////////

// // Below are some sample configurations.
// // You can comment out the default one and uncomment the one you wish to use.
// // You can of course use something different if you like
// // Don't forget to also connect ODrive ISOVDD and ISOGND to Arduino 3.3V/5V and GND.

// // Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// // Note that this is implemented poorly and can lead to wrong data sent or read.
// // pin 8: RX - connect to ODrive TX
// // pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(0, 1);
// unsigned long baudrate = 19200; // Must match what you configure on the ODrive (see docs for details)

// // Teensy 3 and 4 (all versions) - Serial1
// // pin 0: RX - connect to ODrive TX
// // pin 1: TX - connect to ODrive RX
// // See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
// // HardwareSerial& odrive_serial = Serial1;
// // unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)

// // Arduino Mega or Due - Serial1
// // pin 19: RX - connect to ODrive TX
// // pin 18: TX - connect to ODrive RX
// // See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// // HardwareSerial& odrive_serial = Serial1;
// // unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)


// ODriveUART odrive(odrive_serial);

// void setup() {
//   odrive_serial.begin(baudrate);

//   Serial.begin(115200); // Serial to PC
  
//   delay(10);

//   Serial.println("Waiting for ODrive...");
//   /*while (odrive.getState() == AXIS_STATE_UNDEFINED) {
//     delay(100);
//   }*/

//   Serial.println("found ODrive");
  
//   Serial.print("DC voltage: ");
//   Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
//   Serial.println("Enabling closed loop control...");
//   while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
//     odrive.clearErrors();
//     odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
//     delay(10);
//   }
  
//   Serial.println("ODrive running!");
// }

// float phase = 0;
// void loop() {
//   float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

//   float t = 0.001 * millis();
  
//   float phase = t * (TWO_PI / SINE_PERIOD);


//   odrive.setTorque(
//     sin(phase)/*, // position
//     cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)*/
//   );

//   ODriveFeedback feedback = odrive.getFeedback();
//   Serial.print("pos:");
//   Serial.print(feedback.pos);
//   Serial.print(", ");
//   Serial.print("vel:");
//   Serial.print(feedback.vel);
//   Serial.println();
// }