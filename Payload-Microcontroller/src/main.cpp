#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#include <bno-imu.hpp>
#include "pi-communication-test.h"
#include "altimeter.h"

enum flight_phase {
  ON_PAD,
  POST_BOOST,
  DESCENDING,
  LANDED
};

flight_phase current_phase = ON_PAD;

AltimeterData flight_data;

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
      if ( flight_data.altitude_m > /* INSERT POST_BOOST THRESHOLD */ ) {
        current_phase = POST_BOOST;
        Serial.println("LIFTOFF DETECTED");
      }
      
    case POST_BOOST:
      if ( flight_data.altitude_m  < /* INSERT DESCENDING THRESHOLD */ ) {
        current_phase = DESCENDING;
        Serial.println("STARTING CONTROL SYSTEM");
        // Trigger RPi to start capturing pictures
      }
    
    case DESCENDING:
      if ( flight_data.altitude_m  < /* INSERT DESCENDING THRESHOLD */ ) {
        current_phase = LANDED;
        Serial.println("LANDED");
      }
      // Insert final data handling

  }


}
