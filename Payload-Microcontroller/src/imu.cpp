// imu.cpp
//
// contains all logic and control
// related to setting up and getting data
// from the IMU
#include "imu.h"
#include <HardwareSerial.h>

HardwareSerial& IMUSerial = Serial1;

// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// serialNum: the serial number of the teensy connected to the IMU
// outputmode: the output mode for the IMU (0: asynchronous binary, 1: asynchronous ASCII, 2: synchronous binary, 3: synchronous ASCII)  
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200){
    sendUARTCommand("$VNASY,0*4E"); //disable asynchronous data output
    if (baudRate != 115200) {
        sendUARTCommand("$VNWRG,05,baudRate*XX"); //set baud rate
    }
    sendUARTCommand("$VNWRG,06,17,0*XX"); //set output mode. rn its Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements but look into body vs inertial
    // sendUARTCommand("$VNWRG,60"); //add timestamp to data output

    



    return 0;

}


// readIMU: (int) -> (float*)
// reads the asynchronous data stream from RX and returns a pointer to an array of floats
//timeUTC, ypr, angularrate, accel, PosLla, velBody, InsStatus
struct IMUData {
   double timeUTC;
   float ypr[3];
   float angularRate[3];
   float accel[3];
   double posLla[3];
   float velBody[3];
   uint16_t insStatus;
};

IMUData readIMU(uint8_t* buffer) {
   IMUData data;
  
   // 1. Verify Header
   if (buffer[0] != 0xFA) return data;
   uint8_t groupmask = buffer[1];



   }
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


   // 2. Locate the Type Masks (assuming Groups 1, 2, and 6 are active)
   // In a real scenario, you'd check buffer[1] to see how many masks exist.
   uint16_t mask1 = *reinterpret_cast<uint16_t*>(&buffer[2]);
   uint16_t mask2 = *reinterpret_cast<uint16_t*>(&buffer[4]);
   uint16_t mask6 = *reinterpret_cast<uint16_t*>(&buffer[6]);


   // 3. Start pointer at the beginning of the payload
   uint8_t* ptr = &buffer[8];

// --- Process Group 1 (Common) ---
   if (mask1 & (1 << 3)) { // YPR
       std::memcpy(data.ypr, ptr, 12);
       ptr += 12;
   }
   if (mask1 & (1 << 5)) { // Angular Rate
       std::memcpy(data.angularRate, ptr, 12);
       ptr += 12;
   }
   if (mask1 & (1 << 8)) { // Accel
       std::memcpy(data.accel, ptr, 12);
       ptr += 12;
   }


   // --- Process Group 2 (Time) ---
   if (mask2 & (1 << 6)) { // TimeUTC
       // Note: TimeUTC is usually a struct of bytes.
       // If you want a single 'double', you'll need to convert it.
       // For now, let's assume you're just grabbing the raw 8-byte TimeStartup
       // or specific UTC bytes.
       std::memcpy(&data.timeUTC, ptr, 8);
       ptr += 8;
   }


   // --- Process Group 6 (INS) ---
   if (mask6 & (1 << 0)) { // InsStatus
       std::memcpy(&data.insStatus, ptr, 2);
       ptr += 2;
   }
   if (mask6 & (1 << 1)) { // PosLla
       std::memcpy(data.posLla, ptr, 24); // 3 doubles
       ptr += 24;
   }
   if (mask6 & (1 << 3)) { // VelBody
       std::memcpy(data.velBody, ptr, 12);
       ptr += 12;
   }


   return data;
}
  
// IMUErrorCode: () -> (int)
// Does something if we get the $VNERR message from the IMU
int IMUErrorCode(){
    return 0;
}

// sendUARTCommand: (const char*) -> (void)
// sends an ASCII command over UART to the IMU, then waits for an acknowledgment. Also checks checksum
// thanks claude
void sendUARTCommand(const char* command, unsigned int timeout_ms){
    // Send the command over UART
    Serial.print(command);
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    String response = "";
    
    // Read response until timeout
    while(millis() - startTime < timeout_ms){
        if(Serial.available() > 0){
            char c = Serial.read();
            response += c;
            
            // Check if we have at least 2 characters for checksum
            if(response.length() >= 2){
                // Reset timeout on each character received
                startTime = millis();
                
                // Check if message seems complete (you may need to adjust this
                // based on your IMU's protocol - e.g., looking for a terminator)
                // For now, we'll wait a short time for more data
                delay(1 ); // Is this delay necessary? idk
                if(!Serial.available()){
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