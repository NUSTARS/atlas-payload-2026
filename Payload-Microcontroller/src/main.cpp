#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#include <bno-imu.hpp>
#include "pi-communication-test.h"

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin();                 // SDA=18, SCL=19 on Teensy 4.0
  Wire.setClock(400000);        // Fast I2C

  setup_slave();

  Serial.println("set up slave");

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
  
  static unsigned long prevSend = 0;
  if (millis() - prevSend >= 10) { // 100 Hz packet publish
    spi_set_imu_packet(get_acceleration(), get_gyro(), get_quat());
    prevSend = millis();
  }
}