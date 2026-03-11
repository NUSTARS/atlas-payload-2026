#include <Arduino.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
#include <KalmanFilter.h>
#include <bno-imu.hpp>
// #include <imu.h>

// DEFINES =========================================================================
enum flight_phase {
  ON_PAD,
  POST_BOOST,
  ABOVE_ASCENDING_CONTROL_LOCKOUT_THRESHOLD,
  DESCENDING,
  RUN_CONTROLS,
  LANDED
};

#define POST_BOOST_THRESHOLD_M 112
#define ASCENDING_CONTROL_LOCKOUT_THRESHOLD_M 1000
#define DESCENDING_ALTITUDE_DELTA_M 1234
#define RUN_CONTROLS_THRESHOLD_M 300
#define LANDED_THRESHOLD_M 10

#define BUZZER_PIN 17

#define CONTROL_PERIOD_us 1000000 // period of the control interrupt in microseconds

// GLOBALS =========================================================================
float base_altitude_m;
float prev_altitude_m = 0.0; // for use in check for advance to DESCENDING state


flight_phase current_phase = flight_phase::ON_PAD;

AltimeterData flight_data;

KalmanFilter kf;

IMUData imu_data;
bool imu_data_valid = false;

const size_t IMU_FRAME_LEN = 88;
uint8_t imu_rx_buf[IMU_FRAME_LEN];
bool imu_frame_ready = false;

BatteryData battery_data;

IntervalTimer control_timer;


// void pollIMU() {
//   if (Serial1.available() >= IMU_FRAME_LEN) {
//     if (Serial1.peek() == 0xFA) {
//       Serial1.readBytes(imu_rx_buf, IMU_FRAME_LEN);
//       imu_frame_ready = true;
//     } else {
//       Serial1.read();
//     }
//   }
// }

// INTERRUPTS ======================================================================
void controlInterrupt() {
  Serial.println("controls");
  if (!imu_data_valid) return;

  spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
  
  // kalman filter
  float roll_heading = imu_data.ypr[2]; 
  float roll_velocity = imu_data.angularRate[2];
  Eigen::Vector<float,2> z = meas_vector(roll_heading, roll_velocity); // unit conversion inside
  // FIXME update timestep
  kf.predict();
  kf.update(z);
  Eigen::Vector3f x_hat = kf.state();
  
  // only continue if we are running controls
  if (current_phase != RUN_CONTROLS) return;
  /*
  // get pid value
  float pid_result = PIDControl(x_hat[0], x_hat[1]);
  // get feed-forward value
  float ff_result = feedForwardControl(x_hat[2]);
  // command motor
  motorControl(pid_result + ff_result);
  */
}

// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  setup_slave();
  Serial.println("set up teensy as slave");

  bool bat_sensor_setup = initBatSensor();
  Serial.println("set up battery sensor!");
  /*
  if (!bat_sensor_setup) {
    while (1) {
      delay(200);
    }
  }
  */

  // Initialize Altimeter
  bool success_alti_setup = initAltimeter();
  Serial.println("set up Altimeter!");
  /*
  if (!success_alti_setup) {
    while (1) {
      delay(200);
    }
  }
  */
  readAltimeter(flight_data);
  base_altitude_m = flight_data.altitude_m;

  // Initialize IMU
  bool success_bno_setup = initIMU();
  Serial.println("set up BNO085!");
  /*
  if (!success_bno_setup) {
    while (1) {
      delay(200);
    }
  }
  */

  // Initialize Kalman Filter
  float dt0 = CONTROL_PERIOD_us * 1e-6f; // seconds
  // // FIXME read imu and store to
  float roll_heading0 = 0.0;
  float roll_velocity0 = 0.0;
  Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  kf.init(dt0, z0);


  

  // Start IMU data collection + control interrupt
  control_timer.priority(0);
  control_timer.begin(controlInterrupt, CONTROL_PERIOD_us);


  // call buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

// LOOP ============================================================================
void loop() {
  // pollIMU();

  // if (imu_frame_ready) {
  //   uint8_t frame_buf[IMU_FRAME_LEN];
  //   noInterrupts();
  //   memcpy(frame_buf, imu_rx_buf, IMU_FRAME_LEN);
  //   imu_frame_ready = false;
  //   interrupts();

  //   IMUData parsed_imu = readIMU(frame_buf);

  //   noInterrupts();
  //   imu_data = parsed_imu;
  //   imu_data_valid = true;
  //   interrupts();
  // }

  readBatSensor(battery_data);
  // Serial.print("Voltage (V): ");Serial.println(battery_data.voltage_v);
  // Serial.print("Current (mA): ");Serial.println(battery_data.current_ma);
  // Serial.print("Power (mW): ");Serial.println(battery_data.power_mw);
  // Serial.print("Load Voltage (V): ");Serial.println(battery_data.load_voltage_V);
  // Serial.println();

  readAltimeter(flight_data);
  // Serial.print("Temp (C): ");Serial.println(flight_data.temp_C);
  // Serial.print("Pressure (hPa): ");Serial.println(flight_data.pressure_hPa);
  Serial.print("Altitude (meters): ");Serial.println(flight_data.altitude_m);
  // Serial.println();


  readIMU(imu_data);
  // Serial.print("Time (s): "); Serial.println(imu_data.timeUTC, 3);

  // Serial.print("YPR (deg): ");
  // Serial.print(imu_data.ypr[0], 3); Serial.print(", ");
  // Serial.print(imu_data.ypr[1], 3); Serial.print(", ");
  // Serial.println(imu_data.ypr[2], 3);

  // Serial.print("Angular Rate (rad/s): ");
  // Serial.print(imu_data.angularRate[0], 4); Serial.print(", ");
  // Serial.print(imu_data.angularRate[1], 4); Serial.print(", ");
  // Serial.println(imu_data.angularRate[2], 4);

  // Serial.print("Accel (m/s^2): ");
  // Serial.print(imu_data.accel[0], 4); Serial.print(", ");
  // Serial.print(imu_data.accel[1], 4); Serial.print(", ");
  // Serial.println(imu_data.accel[2], 4);

  // Serial.print("LLA (lat, lon, alt): ");
  // Serial.print(imu_data.posLla[0], 6); Serial.print(", ");
  // Serial.print(imu_data.posLla[1], 6); Serial.print(", ");
  ///Serial.println(imu_data.posLla[2], 2);

  // Serial.print("Vel Body (m/s): ");
  // Serial.print(imu_data.velBody[0], 3); Serial.print(", ");
  // Serial.print(imu_data.velBody[1], 3); Serial.print(", ");
  // Serial.println(imu_data.velBody[2], 3);

  // Serial.print("INS Status: ");
  // Serial.println(imu_data.insStatus);

  //Serial.println();


  static unsigned long prevSend = 0;
  if (millis() - prevSend >= 10) { // 100 Hz packet publish
    
    // set values into packet 
    spi_packet_set_voltage(battery_data.voltage_v);

    spi_packet_set_altitude(flight_data.altitude_m);

    spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
    
    spi_packet_set_state(current_phase);

    // build packet
    build_spi_buffer();

    prevSend = millis();
  }

  Serial.println(current_phase);

  float altitude_agl_m = flight_data.altitude_m - base_altitude_m;
  switch (current_phase) {

    case ON_PAD:
      if (altitude_agl_m > POST_BOOST_THRESHOLD_M) {
        current_phase = POST_BOOST;
        Serial.println("LIFTOFF DETECTED");
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, HIGH);

        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
      }
    
    case (flight_phase::POST_BOOST):
      if (altitude_agl_m > ASCENDING_CONTROL_LOCKOUT_THRESHOLD_M) {
        current_phase = flight_phase::ABOVE_ASCENDING_CONTROL_LOCKOUT_THRESHOLD;
      }
      
    case (flight_phase::ABOVE_ASCENDING_CONTROL_LOCKOUT_THRESHOLD):
      if (prev_altitude_m - altitude_agl_m > DESCENDING_ALTITUDE_DELTA_M ) {
        current_phase = flight_phase::DESCENDING;
      }
      prev_altitude_m = altitude_agl_m;

    case DESCENDING:
      if (altitude_agl_m < RUN_CONTROLS_THRESHOLD_M) {
        current_phase = RUN_CONTROLS;
        Serial.println("STARTING CONTROL SYSTEM");
        // Trigger RPi to start capturing pictures
      }
    
    case RUN_CONTROLS:
      if (altitude_agl_m  < LANDED_THRESHOLD_M) {
        current_phase = LANDED;
        Serial.println("LANDED");
      }
    //default:
      
      //Serial.println("Default");
      // Insert final data handling

  }

}

