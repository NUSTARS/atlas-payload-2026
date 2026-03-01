#include <Arduino.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include "pi-communication.h"
#include <imu.h>
#include <KalmanFilter.h>

// DEFINES =========================================================================
enum flight_phase {
  ON_PAD,
  POST_BOOST,
  DESCENDING,
  RUN_CONTROLS,
  LANDED
};


#define POST_BOOST_THRESHOLD_M 50
#define DESCENDING_ALTITUDE_DELTA_M 1234
#define RUN_CONTROLS_THRESHOLD_M 300
#define LANDED_THRESHOLD_M 10

#define BUZZER_PIN 17

#define CONTROL_PERIOD_us 10 // period of the control interrupt in microseconds

// GLOBALS =========================================================================
float prev_altitude_m = 0.0; // for use in check for advance to DESCENDING state

flight_phase current_phase = ON_PAD;

AltimeterData flight_data;

KalmanFilter kf;

IMUData imu_data{};
bool imu_data_valid = false;

size_t IMU_FRAME_LEN = 88;
uint8_t imu_rx_buf[IMU_FRAME_LEN];
bool imu_frame_ready = false;

BatteryData battery_data;

IntervalTimer control_timer;

void pollIMU() {
  if (Serial1.available() >= IMU_FRAME_LEN) {
    if (Serial1.peek() == 0xFA) {
      Serial1.readBytes(imu_rx_buf, IMU_FRAME_LEN);
      imu_frame_ready = true;
    } else {
      Serial1.read();
    }
  }
}
// INTERRUPTS ======================================================================
void controlInterrupt() {
  if (!imu_data_valid) return;

  spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);

  
  // kalman filter
  float roll_heading = imu_data.ypr[2]; 
  float roll_velocity = imu_data.angularRate[2];
  // FIXME update timestep
  kf.predict();
  kf.update(roll_heading, roll_velocity);
  Eigen::Vector3f x_hat = kf.state();
  
  // only continue if we are running controls
  if (current_phase != RUN_CONTROLS) return;

  // get pid value
  float pid_result = PIDControl(x_hat[0], x_hat[1]);
  // get feed-forward value
  float ff_result = feedForwardControl(x_hat[2]);
  // command motor
  motorControl(pid_result + ff_result);
}

// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  setup_slave();
  Serial.println("set up slave");

  bool bat_sensor_setup = initBatSensor();
  Serial.println("set up battery sensor!");
  if (!bat_sensor_setup) {
    while (1) {
      delay(200);
    }
  }

  // Initialize Altimeter
  bool success_alti_setup = initAltimeter();
  Serial.println("set up Altimeter!");
  if (!success_alti_setup) {
    while (1) {
      delay(200);
    }
  }
  
  // Initialize IMU
  initIMU();

  // Initialize Kalman Filter
  float dt0 = CONTROL_PERIOD_us * 1e-6f;
  // FIXME read imu and store to 
  float phi0 = 0.0;
  float omega0 = 0.0;
  // FIXME convert units
  Eigen::Matrix3f P0;



  // Start IMU data collection + control interrupt
  control_timer.priority(0);
  control_timer.begin(controlInterrupt, CONTROL_PERIOD_us);


  // call buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  delay(500);
  digitalWrite(BUZZER_PIN, LOW);

  


  // bool success_bno_setup = setup_bno();
  // Serial.println("set up BNO!");
  // if (!success_bno_setup) {
  //   while (1) {
  //     delay(200);
  //   }
  // }
}

// LOOP ============================================================================



void loop() {
  pollIMU();

  if (imu_frame_ready) {
    uint8_t frame_buf[IMU_FRAME_LEN];
    noInterrupts();
    memcpy(frame_buf, imu_rx_buf, IMU_FRAME_LEN);
    imu_frame_ready = false;
    interrupts();

    IMUData parsed_imu = readIMU(frame_buf);

    noInterrupts();
    imu_data = parsed_imu;
    imu_data_valid = true;
    interrupts();
  }

  readBatSensor(battery_data);
  Serial.print("Voltage (V): ");Serial.println(battery_data.voltage_v);
  Serial.print("Current (mA): ");Serial.println(battery_data.current_ma);
  Serial.print("Power (mW): ");Serial.println(battery_data.power_mw);
  Serial.print("Load Voltage (V): ");Serial.println(battery_data.load_voltage_V);
  Serial.println();


  // update_bno_values();

  readAltimeter(flight_data);
  Serial.print("Temp (C): ");Serial.println(flight_data.temp_C);
  Serial.print("Pressure (hPa): ");Serial.println(flight_data.pressure_hPa);
  Serial.print("Altitude (meters): ");Serial.println(flight_data.altitude_m);
  Serial.println();


  static unsigned long prevSend = 0;
  if (millis() - prevSend >= 10) { // 100 Hz packet publish
    
    // set values into packet 
    spi_packet_set_voltage(battery_data.voltage_v);

    spi_packet_set_altitude(flight_data.altitude_m);
    
    spi_packet_set_state(current_phase);

    // build packet
    build_spi_buffer();

    prevSend = millis();
  }

  // switch (current_phase) {

  //   case ON_PAD:
  //     if (flight_data.altitude_m > ON_PAD) {
  //       current_phase = POST_BOOST;
  //       Serial.println("LIFTOFF DETECTED");
  //     }
      
  //   case POST_BOOST:
  //     if (prev_altitude_m - flight_data.altitude_m > DESCENDING_ALTITUDE_DELTA_M ) {
  //       current_phase = DESCENDING;
  //     }
  //     prev_altitude_m = flight_data.altitude_m;
      

  //   case DESCENDING:
  //     if (flight_data.altitude_m < RUN_CONTROLS_THRESHOLD_M) {
  //       current_phase = RUN_CONTROLS;
  //       Serial.println("STARTING CONTROL SYSTEM");
  //       // Trigger RPi to start capturing pictures
  //     }
    
  //   case RUN_CONTROLS:
  //     if (flight_data.altitude_m  < LANDED_THRESHOLD_M) {
  //       current_phase = LANDED;
  //       Serial.println("LANDED");
  //     }
  //     // Insert final data handling

  // }


}

