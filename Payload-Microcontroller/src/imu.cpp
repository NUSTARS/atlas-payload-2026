// imu.cpp
//
// contains all logic and control
// related to setting up and getting data
// from the IMU
#include "imu.h"
#ifdef ARDUINO
 #include <HardwareSerial.h>
  static HardwareSerial IMUSerial(1);
  HardwareSerial& IMUSerial = Serial1;
#else
  // This part runs on your computer
  #include <iostream> 
  #include <vector>
  #include <cstring>
  #include <cstdint>
#endif



// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// serialNum: the serial number of the teensy connected to the IMU
// outputmode: the output mode for the IMU (0: asynchronous binary, 1: asynchronous ASCII, 2: synchronous binary, 3: synchronous ASCII)  
// returns 1 on successful connection, 0 otherwise
// int initIMU(int baudRate = 115200){
//     sendUARTCommand("$VNASY,0*4E"); //disable asynchronous data output
//     if (baudRate != 115200) {
//         sendUARTCommand("$VNWRG,05,baudRate*XX"); //set baud rate
//     }
//     sendUARTCommand("$VNWRG,06,17,0*XX"); //set output mode. rn its Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements but look into body vs inertial
//     // sendUARTCommand("$VNWRG,60"); //add timestamp to data output

    



//     return 0;

// }


// readIMU: (int) -> (float*)
// reads the asynchronous data stream from RX and returns a pointer to an array of floats
//TimeStartup, ypr, angularrate, accel, PosLla, velNed, InsStatus
struct IMUData {
   double timeStartup;
   float ypr[3];
   float angularRate[3];
   float accel[3];
   double posLla[3];
   float velNed[3];

   uint16_t insStatus;
   
};
IMUData readIMU(uint8_t* buffer) {
    IMUData data = {0};

    // 1. Sync Check
    if (buffer[0] != 0xFA || buffer[1] != 0xFB) return data;

    // 2. GROUP 1 (Common) - This starts at index 22
    // Your hex shows: 48 52 A6 C0 67 00 00 00 (Time) 
    // Followed by: 3A 74 66 C2 A4 CE 3B C1 7C 61 A8 C0 (YPR)
    std::memcpy(&data.timeStartup, &buffer[22], 8);
    std::memcpy(data.ypr,          &buffer[30], 12);

    // 3. THE LONG GAP
    // Your hex has a massive repetition of data and zeros.
    // Based on your specific hex, the INS and IMU data 
    // actually starts much further down.

    // 4. GROUP 6 (INS) 
    // INS Status (2 bytes) - Found at index 134
    // PosLLA (24 bytes)     - Found at index 136
    // VelNed (12 bytes)     - Found at index 160
    std::memcpy(&data.insStatus,   &buffer[134], 2);
    std::memcpy(data.posLla,       &buffer[136], 24);
    std::memcpy(data.velNed,       &buffer[160], 12);

    // 5. GROUP 4 (IMU) - Directly follows Velocity in your stream
    // AngRate (12 bytes) - Starts at 172
    // Accel (12 bytes)   - Starts at 184
    std::memcpy(data.angularRate,  &buffer[172], 12);
    std::memcpy(data.accel,        &buffer[184], 12);

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
//  void sendUARTCommand(const char* command, unsigned int timeout_ms){
//     // Send the command over UART
//     Serial.print(command);
    
//     // Wait for response with timeout
//     unsigned long startTime = millis();
//     String response = "";
    
//     // Read response until timeout
//     while(millis() - startTime < timeout_ms){
//         if(Serial.available() > 0){
//             char c = Serial.read();
//             response += c;
            
//             // Check if we have at least 2 characters for checksum
//             if(response.length() >= 2){
//                 // Reset timeout on each character received
//                 startTime = millis();
                
//                 // Check if message seems complete (you may need to adjust this
//                 // based on your IMU's protocol - e.g., looking for a terminator)
//                 // For now, we'll wait a short time for more data
//                 delay(1 ); // Is this delay necessary? idk
//                 if(!Serial.available()){
//                     break; // No more data coming
//                 }
//             }
//         }
//     }
    
//     // Check if we received a response
//     if(response.length() < 2){
//         // Handle error: no response or response too short
//         Serial.println("Error: No valid response received");
//         return;
//     }
    
//     // Extract checksum (last 2 characters)
//     unsigned int responseLen = response.length();
//     unsigned char receivedChecksum = (unsigned char)response[responseLen - 1];
    
//     // Calculate checksum on all data except the last character
//     unsigned char data[responseLen - 1];
//     for(unsigned int i = 0; i < responseLen - 1; i++){
//         data[i] = (unsigned char)response[i];
//     }
//     unsigned char calculatedChecksum = calculateChecksum(data, responseLen - 1);
    
//     // Verify checksum
//     if(receivedChecksum != calculatedChecksum){
//         Serial.println("Error: Checksum mismatch");
//         Serial.print("Received: 0x");
//         Serial.println(receivedChecksum, HEX);
//         Serial.print("Calculated: 0x");
//         Serial.println(calculatedChecksum, HEX);
//     } else {
//         Serial.println("Command acknowledged successfully");
//     }
// }
void testbinary(){



    //Serial.println("Starting IMU Binary Test...");


    uint8_t debugBuffer[] = {
    0xFA, 0xFB, 0x80, 0x01, 0x09, 0x00, 0x01, 0x00, 0x10, 0xC0, 0x02, 0x00, 0x02, 0x21, 0x13, 0x06,
    0x10, 0x80, 0x02, 0x00, 0x31, 0x00, 0x48, 0x52, 0xA6, 0xC0, 0x67, 0x00, 0x00, 0x00, 0x3A, 0x74,
    0x66, 0xC2, 0xA4, 0xCE, 0x3B, 0xC1, 0x7C, 0x61, 0xA8, 0xC0, 0x48, 0x52, 0xA6, 0xC0, 0x67, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x09, 0x3A, 0x74, 0x66, 0xC2, 0xA4, 0xCE, 0x3B, 0xC1, 0x7C,
    0x61, 0xA8, 0xC0, 0xE1, 0xFA, 0xC7, 0x42, 0xE1, 0xFA, 0xC7, 0x42, 0xE1, 0xFA, 0xC7, 0x42, 0x23,
    0x5C, 0x1E, 0x43, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE1, 0xFA, 0xC7, 0x42, 0xE7, 0xFB, 0x1F, 0x41,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x8A, 0x87, 0x40, 0x79,
    0xBD, 0xBF, 0x41, 0xA9, 0x27, 0x12, 0x41, 0xA1, 0xE7, 0xB1, 0x41, 0x3E, 0xB2, 0x21, 0x42, 0x00,
    0x00, 0x48, 0x44, 0x00, 0x00, 0xC8, 0x43, 0x00, 0x00, 0x48, 0x43, 0x00, 0x00, 0xA0, 0x3E, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3B, 0xEF, 0xDE, 0x43, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x04
};

    // 2. Call your parsing function
    IMUData result = readIMU(debugBuffer);

    // 3. Print the results to see if the data makes sense
std::cout << "Time Startup: " << result.timeStartup << std::endl;
std::cout << "YPR Yaw: " << result.ypr[0] << std::endl;
std::cout << "YPR Pitch: " << result.ypr[1] << std::endl;
std::cout << "YPR Roll: " << result.ypr[2] << std::endl;


std::cout << "AngRate X: " << result.angularRate[0] << std::endl;
std::cout << "AngRate Y: " << result.angularRate[1] << std::endl;
std::cout << "AngRate Z: " << result.angularRate[2] << std::endl;

std::cout << "Pos LLA Lat: " << result.posLla[0] << std::endl;
std::cout << "Pos LLA Lon: " << result.posLla[1] << std::endl;
std::cout << "Pos LLA Alt: " << result.posLla[2] << std::endl;

std::cout << "Accel X: " << result.accel[0] << std::endl;
std::cout << "Accel Y: " << result.accel[1] << std::endl;
std::cout << "Accel Z: " << result.accel[2] << std::endl;

std::cout << "INS Status: " << result.insStatus << std::endl;

std::cout << "VelNed X: " << result.velNed[0] << std::endl;
std::cout << "VelNed Y: " << result.velNed[1] << std::endl;
std::cout << "VelNed Z: " << result.velNed[2] << std::endl;




}

