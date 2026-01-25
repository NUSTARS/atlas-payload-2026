// imu.cpp
//
// contains all logic and control
// related to setting up and getting data
// from the IMU
#include "imu.h"
#include <HardwareSerial.h>

static HardwareSerial IMUSerial(1); // Use Serial1 for IMU communication

// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// serialNum: the serial number of the teensy connected to the IMU
// outputmode: the output mode for the IMU (0: asynchronous binary, 1: asynchronous ASCII, 2: synchronous binary, 3: synchronous ASCII)  
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200, int serialNum){
    sendUARTCommand("$VNASY,0*4E"); //disable asynchronous data output
    if (baudRate != 115200) {
        sendUARTCommand("$VNWRG,05,baudRate*XX"); //set baud rate
    }
    sendUARTCommand("$VNWRG,06,17,0*XX"); //set output mode. rn its Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements but look into body vs inertial
    // sendUARTCommand("$VNWRG,60"); //add timestamp to data output

    



    return 0;

}


// readIMU: (int) -> (float*)
// se
float* readIMU(int syncPin){

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
void sendUARTCommand(const char* command, unsigned int timeout_ms = 1000){
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