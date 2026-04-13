#include <Arduino.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
#include <KalmanFilter.h>
#include <imu.h>
#include <tuning.h>

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

KalmanFilter kf;

IMUData imu_data;

BatteryData battery_data;

IntervalTimer control_timer;

// INTERRUPTS ======================================================================

// 1600 Hz control ISR with reentrancy guard
static volatile bool control_running = false;

static void controlISR() {
    // Skip this tick if control is still running from previous tick
    if (control_running) return;
    control_running = true;

    // Drain IMU serial buffer at high frequency
    serviceIMU();

    // Run control if a new IMU frame has arrived
    if (clearIMUFrameReady() && current_phase == RUN_CONTROLS) {
        if (getLatestIMUData(imu_data)) {
            double time_s = imu_data.timeUTC * 1e-3;

            spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
            
            // debug printout of time and ypr
            // Serial.print("Time (s): "); Serial.println(time_s, 3);
            // Serial.print("YPR (deg): "); Serial.print(imu_data.ypr[0], 2); Serial.print(", "); Serial.print(imu_data.ypr[1], 2); Serial.print(", "); Serial.println(imu_data.ypr[2], 2);
            
            // Kalman filter
            float roll_heading = imu_data.ypr[0];
            float roll_velocity = imu_data.angularRate[0];
            Eigen::Vector<float,2> z = meas_vector(roll_heading, roll_velocity);
            kf.predict();
            kf.update(z);
            Eigen::Vector3f x_hat = kf.state();

            // PID + feedforward control
            float pid_result = PIDControl(roll_heading, roll_velocity);
            float ff_result = feedForwardControl(x_hat[2]);
            float motor_cmd = pid_result + ff_result;
            motorControl(motor_cmd);

            // Tuning telemetry
            tuningSendTelemetry(time_s, roll_heading, roll_velocity, x_hat[2], pid_result, ff_result, motor_cmd);
        }
    }

    control_running = false;
}

// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  // delay(1500);

  setup_slave();
  Serial.println("set up teensy as slave");

  setupControls();

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
  float dt0 = 1/100.0f; // seconds
  // // FIXME read imu and store to
  float roll_heading0 = 0.0;
  float roll_velocity0 = 0.0;
  Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  kf.init(dt0, z0);

  // Initialize 1600 Hz control ISR with highest priority
  control_timer.begin(controlISR, 625);  // 625 microseconds = 1600 Hz
  // Set to priority 0 (highest) to preempt all other interrupts
  control_timer.priority(0);


  // call buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);  

  delay(500);
  digitalWrite(BUZZER_PIN, LOW);

  if (current_phase == TUNING) {  
    tuningSetup();
  }
}

// LOOP ============================================================================
void loop() {
  tuningServiceUsbCommands();

  // Serial.print("Time (s): "); Serial.println(imu_data.timeUTC, 3);

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

  // // Serial.print("Current Phase: "); Serial.println(current_phase);

  // setVel();

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