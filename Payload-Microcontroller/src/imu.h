// imu.h

// initIMU: (int, int, int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// RXPin: the RX pin for serial communication
// TXPin: the TX pin for serial communication
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200, int RXPin, int TXPin);


// readIMU: (int) -> (float*)
// reads the asynchronous data stream from RX and returns a pointer to an array of floats
float* readIMU();

// closeIMU: () -> (int)
// closes the connection to the IMU
// returns 1 on successful close, 0 otherwise
int closeIMU();

// IMUErrorCode: () -> (int)
// Does something if we get the $VNERR message from the IMU
int IMUErrorCode();

// sendUARTCommand: (const char*) -> (void)
// sends an ASCII command over UART to the IMU
void sendUARTCommand(const char* command);

// Things we will have to do
/**
 * == init function ==
 * set up uart communication (?)
 * Configure IMU settings
 * configure asynchronous data reading (should use binary bc fastest?)
 * 
 * 
 * == parse function ==
 * parse data
 * package data into array
 * do that a lot
 * 
 *
 * == close function ==
 * close connection to IMU
 */