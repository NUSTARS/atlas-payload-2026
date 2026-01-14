// imu.cpp
//
// contains all logic and control
// related to setting up and getting data
// from the IMU



// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// RXPin: the RX pin for serial communication
// TXPin: the TX pin for serial communication
// outputmode: the output mode for the IMU (0: asynchronous binary, 1: asynchronous ASCII, 2: synchronous binary, 3: synchronous ASCII)  
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200, int RXPin, int TXPin){
    sendUARTCommand("$VNASY,0*4E"); //disable asynchronous data output
    sendUARTCommand("$VNWRG,05,baudRate*XX"); //set baud rate
    sendUARTCommand("$VNWRG,17,0*XX"); //set output mode. rn its Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements but look into body vs inertial
    sendUARTCommand("$VNWRG,60); //add timestamp to data output
    return 0;

}


// readIMU: (int) -> (float*)
// reads the asynchronous data stream from RX and returns a pointer to an array of floats
float* parseIMU(){

}

// closeIMU: () -> (int)
// closes the connection to the IMU
// returns 1 on successful close, 0 otherwise
int closeIMU(){
    return 0;
}

// IMUErrorCode: () -> (int)
// Does something if we get the $VNERR message from the IMU
int IMUErrorCode(){
    return 0;
}

// sendUARTCommand: (const char*) -> (void)
// sends an ASCII command over UART to the IMU, then waits for an acknowledgment. Also checks checksum
void sendUARTCommand(const char* command){

}