// imu.h

// initIMU: (int) -> (int)
// initializes the IMU by writing ASCII commands over serial to the IMU
// baudRate: the baud rate for serial communication, defaults at 115200
// returns 1 on successful connection, 0 otherwise
int initIMU(int baudRate = 115200);
struct IMUData {
  bool hasTimeStartup = false;
  uint64_t timeStartup = 0;

  bool hasYpr = false;
  float yaw = 0, pitch = 0, roll = 0;

  bool hasAngularRate = false;
  float angRateX = 0, angRateY = 0, angRateZ = 0;

  bool hasAccel = false;
  float accelX = 0, accelY = 0, accelZ = 0;

  bool hasPosLla = false;
  double lat = 0, lon = 0, alt = 0;

  bool hasVelNed = false;
  float velN = 0, velE = 0, velD = 0;

  bool hasInsStatus = false;
  uint16_t insStatus = 0;
};

// decode a full packet you already have in memory (unit-testable)
bool decodeVNPacket(const uint8_t* data, size_t len, IMUData& out);


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