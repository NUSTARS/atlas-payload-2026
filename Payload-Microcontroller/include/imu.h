// imu.h

#include <Arduino.h>

// initIMU: (int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200);


// readIMU: (uint8_t*) -> (IMUData)
// reads the asynchronous data stream from buffer and returns parsed IMU data
struct IMUData {
	double timeUTC;
	float ypr[3];
	float angularRate[3];
	float accel[3];
	double posLla[3];
	float velBody[3];
	uint16_t insStatus;
};

IMUData readIMU(uint8_t* buffer);

// closeIMU: () -> (int)
// closes the connection to the IMU
// returns 1 on successful close, 0 otherwise
int closeIMU();

// IMUErrorCode: () -> (int)
// Does something if we get the $VNERR message from the IMU
int IMUErrorCode();

// sendUARTCommand: (const char*) -> (void)
// sends an ASCII command over UART to the IMU
void sendUARTCommand(const char* command, unsigned int timeout_ms = 1000);

// attachIMUTriggerISR: (uint8_t, void (*)()) -> (bool)
// attaches an ISR that fires when the specified pin is pulled high
// returns true if the pin supports interrupts and the ISR was attached
bool attachIMUTriggerISR(uint8_t pin, void (*isr)());


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



//  other notes to self:
/**
 * We should use coning and sculling if our control loop rate is a good bit less than the IMU rate (which it is)
 * look into binary outputs - maybe faster
 * make sure to do ahrs if gnss is not available (VNWRG 67)
 * In general I should be doing the 3.X basic config commands for all components on the VN
 * Also decide what to do about the SyncIn rate and skip
 * 
 */
