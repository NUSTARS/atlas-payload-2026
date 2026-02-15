#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>


#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
#include <bno-imu.hpp>
#include "pi-communication-test.h"
#include <altimeter.h>
#include <KalmanFilter.h>
#include <imu.h>

// DEFINES =========================================================================
enum flight_phase {
  ON_PAD,
  POST_BOOST,
  DESCENDING,
  RUN_CONTROLS,
  LANDED
};
// FIXME add correct thresholds
#define POST_BOOST_THRESHOLD_M 50
#define DESCENDING_ALTITUDE_DELTA_M 1234
#define RUN_CONTROLS_THRESHOLD_M 300
#define LANDED_THRESHOLD_M 10


// GLOBALS =========================================================================
float prev_altitude_m = 0.0; // for use in check for advance to DESCENDING state

flight_phase current_phase = ON_PAD;

AltimeterData flight_data;

KalmanFilter kf;

IMUData imu_data;

// INTERRUPTS ======================================================================
void controlInterrupt() {
  // read data
  uint8_t* buffer = {}; // FIXME need to get actual buffer of IMU data
  imu_data = readIMU(buffer);

  // only continue if we are running controls
  if (current_phase != RUN_CONTROLS) return;
  // kalman filter    TODO determine if using doubles
  double roll_heading = (double)imu_data.ypr[2]; 
  double roll_velocity = (double)imu_data.angularRate[2];
  // FIXME update timestep
  kf.update(roll_heading, roll_velocity);
  kf.predict();
  Eigen::Vector3d state = kf.state();
  
  


  // get pid value
  float pid_result = PIDControl((float)state[0], (float)state[1]);
  // get feed-forward value
  float ff_result = feedForwardControl((float)state[2]);
  // command motor
  motorControl(pid_result + ff_result);
}

// SETUP ===========================================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin();                 // SDA=18, SCL=19 on Teensy 4.0
  Wire.setClock(400000);        // Fast I2C

  setup_slave();

  Serial.println("set up slave");

  // Initialize Altimeter
  bool success_alti_setup = initAltimeter();

  Serial.println("set up Altimeter!");

  if (!success_alti_setup) {
    while (1) {
      delay(200);
    }
  }

  bool success_bno_setup = setup_bno();

  Serial.println("set up BNO!");

  if (!success_bno_setup) {
    while (1) {
      delay(200);
    }
  }

}

// LOOP ============================================================================

void loop() {

  update_bno_values();
  readAltimeter(flight_data);

  static unsigned long prevSend = 0;
  if (millis() - prevSend >= 10) { // 100 Hz packet publish
    spi_set_imu_packet(get_acceleration(), get_gyro(), get_quat());
    prevSend = millis();
  }

  switch (current_phase) {

    case ON_PAD:
      if (flight_data.altitude_m > ON_PAD) {
        current_phase = POST_BOOST;
        Serial.println("LIFTOFF DETECTED");
      }
      
    case POST_BOOST:
      if (prev_altitude_m - flight_data.altitude_m > DESCENDING_ALTITUDE_DELTA_M ) {
        current_phase = DESCENDING;
      }
      prev_altitude_m = flight_data.altitude_m;
      

    case DESCENDING:
      if (flight_data.altitude_m < RUN_CONTROLS_THRESHOLD_M) {
        current_phase = RUN_CONTROLS;
        Serial.println("STARTING CONTROL SYSTEM");
        // Trigger RPi to start capturing pictures
      }
    
    case RUN_CONTROLS:
      if (flight_data.altitude_m  < LANDED_THRESHOLD_M) {
        current_phase = LANDED;
        Serial.println("LANDED");
      }
      // Insert final data handling

  }


}
