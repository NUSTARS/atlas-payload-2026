#include <Arduino.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
#include <KalmanFilter.h>
//#include <bno-imu.hpp>
#include <imu.h>
#include <tuning.h>

// DEFINES =========================================================================
enum flight_phase {
  LAUNCH_PAD, // = 0
  BOOST,
  APOGEE,
  RUN_CONTROLS,
  LANDED
};

#define BOOST_DETECTION_HEIGHT_M 64 // 50 feet 
#define APOGEE_DETECTION_HEIGHT_M 1980 // 6500 feet 
#define RUN_CONTROLS_HEIGHT_M 198 // 650 feet 

#define HEIGHT_LOCKOUT_CONTROLS_M 3
#define TIME_LOCKOUT_CONTROLS_MILLIS 45000 

#define TORQUE_APPLYING_PERIOD_MILLIS 4000

// #define BOOST_DETECTION_HEIGHT_M 0.5 // 50 feet 
// #define APOGEE_DETECTION_HEIGHT_M 0.9 // 6500 feet 
// #define RUN_CONTROLS_HEIGHT_M 0.5 // 650 feet 

// #define HEIGHT_LOCKOUT_CONTROLS_M 0
// #define TIME_LOCKOUT_CONTROLS_MILLIS 10000

// #define TORQUE_APPLYING_PERIOD_MILLIS 2000

#define BUZZER_PIN 23

// GLOBALS =========================================================================
float prev_altitude_m = 0.0; // for use in check for advance to DESCENDING state

float TORQUE_EXERTING = 0.9;
bool TORQUE_SIGN_POS = true;


int current_phase;

AltimeterData flight_data;

KalmanFilter kf;

IMUData imu_data;

BatteryData battery_data;

IntervalTimer control_timer;

unsigned long startedControlsTime;

long long counter = 0;

// INTERRUPTS ======================================================================

// control interrupt has been deleted and that stuff has moved to main loop

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
            // Serial.print( "YPR (deg): "); Serial.print(imu_data.ypr[0], 2); Serial.print(", "); Serial.print(imu_data.ypr[1], 2); Serial.print(", "); Serial.println(imu_data.ypr[2], 2);
            
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

            if (!tuningHandshakeAlive()) {
              // Deadman watchdog: host heartbeat missing, force safe output.
              motor_cmd = 0.0f;
            }

            motorControl(motor_cmd);

            // Tuning telemetry
            tuningSendTelemetry(time_s, roll_heading, roll_velocity, x_hat[2], pid_result, ff_result, motor_cmd);
        }
    }

    control_running = false;
}



// void controlInterrupt() {

//   if (readIMU(imu_data)) {
//     // debug printout of time and YPR of newest IMU packet
//     double time_s = imu_data.timeUTC * 1e-3;
//     Serial.print("Time: "); Serial.println(time_s, 3);
//     Serial.print("YPR: ");
//     Serial.print(imu_data.ypr[0], 4); Serial.print(", ");
//     Serial.print(imu_data.ypr[1], 4); Serial.print(", ");
//     Serial.println(imu_data.ypr[2], 4);

//     // spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
    
//     // kalman filter
//     float roll_heading = imu_data.ypr[2]; 
//     float roll_velocity = imu_data.angularRate[2];
//     Eigen::Vector<float,2> z = meas_vector(roll_heading, roll_velocity); // unit conversion inside

//     uint32_t ts_us = micros();
//     kf.predict();
//     Eigen::Vector3f predx = kf.state();
//     Serial.print("predx:\t");
//     Serial.print(predx(0), 6); Serial.print("\t");
//     Serial.println(predx(1), 6);
//     kf.update(z);
//     uint32_t tf_us = micros();

//     Eigen::Vector3f x_hat = kf.state();
//     Serial.print("updx:\t");
//     Serial.print(x_hat(0), 6); Serial.print("\t");
//     Serial.print(x_hat(1), 6); Serial.print("\t");
//     Serial.println(x_hat(2), 6);

//     auto y = kf.innovation();
//     Serial.print("y:  \t");
//     Serial.print(y(0), 6); Serial.print("\t");
//     Serial.println(y(1), 6);
//     Serial.print("Kalman Time (us): "); Serial.println(tf_us - ts_us); 
//     Serial.println("--");

//     // FIXME update timestep
//     // kf.predict();
//     // kf.update(z);
//     // Eigen::Vector3f x_hat = kf.state();
//     // Serial.print("Meas:\t");
//     // Serial.print(roll_heading,6); Serial.print("\t"); Serial.println(roll_velocity,6);
//     // Serial.print("xhat:\t");
//     // Serial.print(x_hat(0),6); Serial.print("\t"); Serial.print(x_hat(1),6); Serial.print("\t"); Serial.println(x_hat(2),6);
    


    
//     // only continue if we are running controls
//     if (current_phase != RUN_CONTROLS) return;

//     // get pid value
//     float pid_result = PIDControl(x_hat[0], x_hat[1]);
//     // get feed-forward value
//     float ff_result = feedForwardControl(x_hat[2]);
//     // command motor
//     motorControl(pid_result + ff_result);
//   }
// }

void buzz() {
  digitalWrite(BUZZER_PIN, HIGH);

  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}


// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  // delay(1500);
  tuningServiceUsbCommands();
  setupControls();
  control_timer.begin(controlISR, 150000); // every 0.15 seconds
  setup_slave();
  // Serial.println("set up teensy as slave");

  bool bat_sensor_setup = initBatSensor();
  // Serial.println("set up battery sensor!");
  // if (!bat_sensor_setup) {
  //   while (1) {
  //     delay(200);
  //   }
  // }

  // Initialize Altimeter
  bool success_alti_setup = initAltimeter();
  // Serial.println("set up Altimeter!");
  // if (!success_alti_setup) {
  //   while (1) {
  //     delay(200);
  //   }
  // }

  // Initialize serial IMU (VectorNav on Serial2)
  bool success_imu_setup = initIMU();
  // if (!success_imu_setup) {
  //   while (1) {
  //     delay(200);
  //   }
  // }
  // Serial.println("set up serial IMU on Wire1!");

  // Initialize Kalman Filter
  // float dt0 = 1/100.0f; // seconds
  // // // FIXME read imu and store to
  // float roll_heading0 = 0.0;
  // float roll_velocity0 = 0.0;
  // Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  // kf.init(dt0, z0);

  // call buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  // buzz();

  current_phase = LAUNCH_PAD;

  // clear errors 
  sendCmd("w axis0.error 0");
  

  // control_timer.priority(0);
  // control_timer.begin(controlInterrupt, CONTROL_PERIOD_us);
}

// LOOP ============================================================================
void loop() {
  //Serial.println("hello world");
  // Serial.println(queryODrive("r axis0.active_errors"));
  // sendCmd("u 0\n");
  // sendCmd("w axis0.error 0");

  // readIMU(imu_data);
  // readBatSensor(battery_data);
  // Serial.print("Voltage (V): ");Serial.println(battery_data.voltage_v);
  // Serial.print("Current (mA): ");Serial.println(battery_data.current_ma);
  // Serial.print("Power (mW): ");Serial.println(battery_data.power_mw);
  // Serial.print("Load Voltage (V): ");Serial.println(battery_data.load_voltage_V);
  // Serial.println();

  // readAltimeter(flight_data);
  static float base_altitude_m = flight_data.altitude_m;
  // Serial.print("Temp (C): ");Serial.println(flight_data.temp_C);
  // Serial.print("Pressure (hPa): ");Serial.println(flight_data.pressure_hPa);
  // Serial.print("Altitude (meters): ");Serial.println(flight_data.altitude_m);
  // Serial.println();

  // Serial.print("Time (s): "); Serial.println(imu_data.timeUTC, 3);

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

  // Serial.print("Current Phase: "); Serial.println(current_phase);

  // LAUNCH_PAD,
  // BOOST,
  // APOGEE,
  // RUN_CONTROLS,
  // LANDED

  float altitude_agl_m = flight_data.altitude_m - base_altitude_m;
  Serial.print("Altitude: "); Serial.println(altitude_agl_m);
  switch (current_phase) {
    case LAUNCH_PAD:

      if (altitude_agl_m > BOOST_DETECTION_HEIGHT_M) {
        Serial.println("BOOST DETECTED");
        current_phase = BOOST;
        // Serial.println("LIFTOFF DETECTED");
        // buzz();
        delay(1500);

      }
      break;
    
    case BOOST:
      // heartbeat for the ODrive
      
      if (altitude_agl_m > APOGEE_DETECTION_HEIGHT_M) {
        Serial.println("APOGEE DETECTED");
        current_phase = APOGEE;
        // buzz();
        delay(1500);
      }
      break;
      
    case APOGEE:

      if (altitude_agl_m < RUN_CONTROLS_HEIGHT_M) {
        Serial.println("CONTROLS RUNNING");
        current_phase = RUN_CONTROLS;
        startedControlsTime = millis();
        // buzz();
        delay(1500);
      }
      break;
      

    case RUN_CONTROLS:
      /*static unsigned long prevCmdSent = millis();
      setTorque((float) (TORQUE_EXERTING * ((((int) TORQUE_SIGN_POS) << 1) - 1)));

      // SEND CMD TO ODRIVE EVERY 2 SEC
      if (millis() - prevCmdSent >= TORQUE_APPLYING_PERIOD_MILLIS) { // 0.5 Hz
        TORQUE_SIGN_POS = !TORQUE_SIGN_POS;

        prevCmdSent = millis();
      }*/

      if (altitude_agl_m < HEIGHT_LOCKOUT_CONTROLS_M || millis() - startedControlsTime > TIME_LOCKOUT_CONTROLS_MILLIS) {
        current_phase = LANDED;
        Serial.println("LANDING DETECTED");
        // buzz();
        delay(1500);
      }
      break;
    
    case LANDED:
      // send command to the ODrive to stop spinning
      setTorque(0.0);
      break;


    //default:
      
      //Serial.println("Default");
      // Insert final data handling

  }

}

