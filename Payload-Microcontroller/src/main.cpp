#include <Arduino.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
<<<<<<< HEAD
// #include <KalmanFilter.h>
=======
#include <KalmanFilter.h>
>>>>>>> 8e3101f3382702e183994b8e6453d997fa26f2b8
#include <imu.h>

// DEFINES =========================================================================
enum flight_phase {
  ON_PAD,
  POST_BOOST,
  ABOVE_ASCENDING_CONTROL_LOCKOUT_THRESHOLD,
  DESCENDING,
  RUN_CONTROLS,
  LANDED,
  TUNING
};

#define POST_BOOST_THRESHOLD_M 112
#define ASCENDING_CONTROL_LOCKOUT_THRESHOLD_M 1000
#define DESCENDING_ALTITUDE_DELTA_M 1234
#define RUN_CONTROLS_THRESHOLD_M 300
#define LANDED_THRESHOLD_M 10

#define BUZZER_PIN 14

// GLOBALS =========================================================================
float base_altitude_m;
float prev_altitude_m = 0.0; // for use in check for advance to DESCENDING state


flight_phase current_phase = flight_phase::RUN_CONTROLS;

AltimeterData flight_data;

// KalmanFilter kf;

IMUData imu_data;

BatteryData battery_data;

<<<<<<< HEAD
=======
IntervalTimer control_timer;

// INTERRUPTS ======================================================================

// control interrupt has been deleted and that stuff has moved to main loop

>>>>>>> 8e3101f3382702e183994b8e6453d997fa26f2b8
// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  setup_slave();
  Serial.println("set up teensy as slave");

  // bool bat_sensor_setup = initBatSensor();
  // Serial.println("set up battery sensor!");
  /*
  if (!bat_sensor_setup) {
    while (1) {
      delay(200);
    }
  }
  */

  // Initialize Altimeter
  // bool success_alti_setup = initAltimeter();
  // Serial.println("set up Altimeter!");
  // if (!success_alti_setup) {
  //   while (1) {
  //     delay(200);
  //   }
  // }
  // readAltimeter(flight_data);
  // base_altitude_m = flight_data.altitude_m;

  // Initialize serial IMU (VectorNav on Serial2)
  initIMU();
  Serial.println("set up serial IMU on Serial2!");

  // Initialize Kalman Filter
<<<<<<< HEAD
  // float dt0 = CONTROL_PERIOD_us * 1e-6f; // seconds
  // float roll_heading0 = 0.0;
  // float roll_velocity0 = 0.0;
  // Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  // kf.init(dt0, z0);
=======
  float dt0 = 1/100.0f; // seconds
  // // FIXME read imu and store to
  float roll_heading0 = 0.0;
  float roll_velocity0 = 0.0;
  Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  kf.init(dt0, z0);
>>>>>>> 8e3101f3382702e183994b8e6453d997fa26f2b8

  // call buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}
int counter = 0;
// LOOP ============================================================================
void loop() {
  // Service the IMU serial RX buffer and update the latest packet snapshot.
  serviceIMU();

  // Run control immediately when a new IMU frame is published.
  static uint32_t last_seq = 0;
  uint32_t now_seq = getIMUSequence();
  // Serial.print('Here');

  if (now_seq != last_seq) {
    last_seq = now_seq;
    if (getLatestIMUData(imu_data) && current_phase == RUN_CONTROLS) {
      // debug printout of time and YPR of newest IMU packet
      double time_s = imu_data.timeUTC * 1e-3;
      Serial.print("Time: "); Serial.println(time_s, 3);
      Serial.print("YPR: ");
      Serial.print(imu_data.ypr[0], 4); Serial.print(", ");
      Serial.print(imu_data.ypr[1], 4); Serial.print(", ");
      Serial.println(imu_data.ypr[2], 4);

      
      spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
      
      // kalman filter
      float roll_heading = imu_data.ypr[2]; 
      float roll_velocity = imu_data.angularRate[2];
      Eigen::Vector<float,2> z = meas_vector(roll_heading, roll_velocity); // unit conversion inside

      uint32_t ts_us = micros();
      kf.predict();
      Eigen::Vector3f predx = kf.state();
      Serial.print("predx:\t");
      Serial.print(predx(0), 6); Serial.print("\t");
      Serial.println(predx(1), 6);
      kf.update(z);
      uint32_t tf_us = micros();

      Eigen::Vector3f x_hat = kf.state();
      Serial.print("updx:\t");
      Serial.print(x_hat(0), 6); Serial.print("\t");
      Serial.print(x_hat(1), 6); Serial.print("\t");
      Serial.println(x_hat(2), 6);

      auto y = kf.innovation();
      Serial.print("y:  \t");
      Serial.print(y(0), 6); Serial.print("\t");
      Serial.println(y(1), 6);
      Serial.print("Kalman Time (us): "); Serial.println(tf_us - ts_us); 
      Serial.println("--");


      // FIXME update timestep
      // kf.predict();
      // kf.update(z);
      // Eigen::Vector3f x_hat = kf.state();
      // Serial.print("Meas:\t");
      // Serial.print(roll_heading,6); Serial.print("\t"); Serial.println(roll_velocity,6);
      // Serial.print("xhat:\t");
      // Serial.print(x_hat(0),6); Serial.print("\t"); Serial.print(x_hat(1),6); Serial.print("\t"); Serial.println(x_hat(2),6);
      


      
      // only continue if we are running controls
      if (current_phase != RUN_CONTROLS) return;

      // get pid value
      float pid_result = PIDControl(x_hat[0], x_hat[1]);
      // get feed-forward value
      float ff_result = feedForwardControl(x_hat[2]);
      // command motor
      motorControl(pid_result + ff_result);
    }
  }

  // Debug output: very sparse to avoid blocking the 50Hz control path



  // readBatSensor(battery_data);
  // Serial.print("Voltage (V): ");Serial.println(battery_data.voltage_v);
  // Serial.print("Current (mA): ");Serial.println(battery_data.current_ma);
  // Serial.print("Power (mW): ");Serial.println(battery_data.power_mw);
  // Serial.print("Load Voltage (V): ");Serial.println(battery_data.load_voltage_V);
  // Serial.println();

  // readAltimeter(flight_data);
  // Serial.print("Temp (C): ");Serial.println(flight_data.temp_C);
<<<<<<< HEAD
  // // Serial.print("Pressure (hPa): ");Serial.println(flight_data.pressure_hPa);
  // Serial.print("Altitude (meters): ");Serial.println(flight_data.altitude_m);
  // // Serial.println();
=======
  // Serial.print("Pressure (hPa): ");Serial.println(flight_data.pressure_hPa);
  // Serial.print("Altitude (meters): ");Serial.println(flight_data.altitude_m);
  // Serial.println();


  // Serial.print("Time (s): "); Serial.println(imu_data.timeUTC, 3);
>>>>>>> 8e3101f3382702e183994b8e6453d997fa26f2b8

  // static unsigned long prevSend = 0;
  // if (millis() - prevSend >= 10) { // 100 Hz packet publish
    
  //   // set values into packet 
  //   spi_packet_set_voltage(battery_data.voltage_v);

  //   spi_packet_set_altitude(flight_data.altitude_m);

  //   spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
    
  //   spi_packet_set_state(current_phase);

  //   // build packet
  //   build_spi_buffer();

  //   prevSend = millis();
  // }

<<<<<<< HEAD
  // Serial.print("Current Phase: "); Serial.println(current_phase);
=======
  // // Serial.print("Current Phase: "); Serial.println(current_phase);

  // setVel();
>>>>>>> 8e3101f3382702e183994b8e6453d997fa26f2b8

  // float altitude_agl_m = flight_data.altitude_m - base_altitude_m;
  // switch (current_phase) {

  //   case ON_PAD:
  //     if (altitude_agl_m > POST_BOOST_THRESHOLD_M) {
  //       current_phase = POST_BOOST;
  //       Serial.println("LIFTOFF DETECTED");
  //       pinMode(BUZZER_PIN, OUTPUT);
  //       digitalWrite(BUZZER_PIN, HIGH);

  //       delay(500);
  //       digitalWrite(BUZZER_PIN, LOW);
  //     }
    
  //   case (flight_phase::POST_BOOST):
  //     if (altitude_agl_m > ASCENDING_CONTROL_LOCKOUT_THRESHOLD_M) {
  //       current_phase = flight_phase::ABOVE_ASCENDING_CONTROL_LOCKOUT_THRESHOLD;
  //     }
      
  //   case (flight_phase::ABOVE_ASCENDING_CONTROL_LOCKOUT_THRESHOLD):
  //     if (prev_altitude_m - altitude_agl_m > DESCENDING_ALTITUDE_DELTA_M ) {
  //       current_phase = flight_phase::DESCENDING;
  //     }
  //     prev_altitude_m = altitude_agl_m;

  //   case DESCENDING:
  //     if (altitude_agl_m < RUN_CONTROLS_THRESHOLD_M) {
  //       current_phase = RUN_CONTROLS;
  //       Serial.println("STARTING CONTROL SYSTEM");
  //       // Trigger RPi to start capturing pictures
  //     }
    
  //   case RUN_CONTROLS:
  //     if (altitude_agl_m  < LANDED_THRESHOLD_M) {
  //       current_phase = LANDED;
  //       Serial.println("LANDED");
  //     }
  //   //default:
      
  //     //Serial.println("Default");
  //     // Insert final data handling

  // }

}

