// imu.cpp
//
// contains all logic and control
// related to setting up and getting data
// from the IMU
#include "imu.h"
#include <Arduino.h>
#include <HardwareSerial.h>


// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// returns 1 on successful connection, 0 otherwise
int initIMU(){
    sendUARTCommand("$VNASY,0*4E"); //disable asynchronous data output
    sendUARTCommand("$VNWRG,06,17,0*XX"); //set output mode. rn its Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements but look into body vs inertial
    // sendUARTCommand("$VNWRG,60"); //add timestamp to data output

    



    return 0;

}


// readIMU: (int) -> (float*)
// reads the asynchronous data stream from RX and returns a pointer to an array of floats
//timeUTC, ypr, angularrate, accel, PosLla, velBody, InsStatus
IMUData readIMU(uint8_t* buffer) {
   IMUData data;
  
   // 1. Verify Header
   if (buffer[0] != 0xFA) return data;
   uint8_t groupmask = buffer[1];
   


   
   // 2. Map the masks (they appear in order of group number)
   // We'll store them in an array so we can access them easily
   uint16_t masks[8] = {0};
   int currentMaskIdx = 0;
   for (int i = 0; i < 8; i++) {
       if (groupmask & (1 << i)) {
           // Grab the 2-byte mask and move the index
           std::memcpy(&masks[i], &buffer[2 + (currentMaskIdx * 2)], 2);
           currentMaskIdx++;
       }
   }
   // 3. Where does the payload start?
   uint8_t* ptr = &buffer[2 + (currentMaskIdx * 2)];
   //  Process Group 1 (Common) 

   if (masks[0] & (1 << 3)) { // YPR
       std::memcpy(data.ypr, ptr, 12);
       ptr += 12;
   }
   if (masks[0] & (1 << 5)) { // Angular Rate
       std::memcpy(data.angularRate, ptr, 12);
       ptr += 12;
   }
   if (masks[0] & (1 << 8)) { // Accel
       std::memcpy(data.accel, ptr, 12);
       ptr += 12;
   }

   // Process Group 2 (Time)

   if (masks[1] & (1 << 6)) { // TimeUTC
       std::memcpy(&data.timeUTC, ptr, 8);
       ptr += 8;
   }
   // test 1
   // Process Group 6 (INS)

   if (masks[5] & (1 << 0)) { // InsStatus
       std::memcpy(&data.insStatus, ptr, 2);
       ptr += 2;
   }
   if (masks[5] & (1 << 1)) { // PosLla
       std::memcpy(data.posLla, ptr, 24); 
       ptr += 24;
   }
   if (masks[5] & (1 << 3)) { // VelBody
       std::memcpy(data.velBody, ptr, 12);
       ptr += 12;
   }

   return data;
}
// closeIMU: () -> (int)
// closes the connection to the IMU
// returns 1 on successful close, 0 otherwise
int closeIMU(){
    // lowkey this might not be necessary
    return 0;
}

// IMUErrorCode: () -> (int)
// Does something if we get the $VNERR message from the IMU
int IMUErrorCode(){
    return 0;
}

// sendUARTCommand: (const char*) -> (void)
// sends an ASCII command over UART to the IMU, then waits for an acknowledgment. Also checks checksum
// thanks claude
int sendUARTCommand(const char* command, unsigned int timeout_ms){
    // Send the command over UART
    Serial1.print(command);
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    String response = "";
    
    // Read response until timeout
    while(millis() - startTime < timeout_ms){
        if(Serial1.available() > 0){
            char c = Serial1.read();
            response += c;
            
            // Check if we have at least 2 characters for checksum
            if(response.length() >= 2){
                // Reset timeout on each character received
                startTime = millis();
                
                if(!Serial1.available()){
                    break; // No more data coming
                }
            }
        }
    }
    
    // Check if we received a response
    if(response.length() < 2){
        // Handle error: no response or response too short
        Serial.println("Error: No valid response received");
        return;
    }
    
    // Extract checksum (last 2 characters)
    unsigned int responseLen = response.length();
    unsigned char receivedChecksum = (unsigned char)response[responseLen - 1];
    
    // Calculate checksum on all data except the last character
    unsigned char data[responseLen - 1];
    for(unsigned int i = 0; i < responseLen - 1; i++){
        data[i] = (unsigned char)response[i];
    }
    unsigned char calculatedChecksum = calculateChecksum(data, responseLen - 1);
    
    // Verify checksum
    if(receivedChecksum != calculatedChecksum){
        Serial.println("Error: Checksum mismatch");
        Serial.print("Received: 0x");
        Serial.println(receivedChecksum, HEX);
        Serial.print("Calculated: 0x");
        Serial.println(calculatedChecksum, HEX);
    } else {
        Serial.println("Command acknowledged successfully");
    }
}

// attachIMUTriggerISR: (uint8_t, void (*)()) -> (bool)
// attaches an ISR that fires when the specified pin is pulled high
// returns true if the pin supports interrupts and the ISR was attached
bool attachIMUTriggerISR(uint8_t pin, void (*isr)()){
    if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT) {
        return false;
    }

    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
    return true;
}