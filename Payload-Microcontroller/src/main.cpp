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
  LAUNCH_PAD, // = 0
  BOOST,
  APOGEE,
  RUN_CONTROLS,
  LANDED
};

//
// set up configuration for phase detection
//
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

//
// VN200 configurations
//
#define VN_TRIGGER_PIN 22
#define VN_TRIGGER_PULSE_US 5
#define IMU_FRAME_TIMEOUT_MS 10
#define CONTROL_ISR_PERIOD_US 100000

#define BUZZER_PIN 23

// GLOBALS =========================================================================
int current_phase;

AltimeterData flight_data;

KalmanFilter kf;

IMUData imu_data;

BatteryData battery_data;

IntervalTimer control_timer;

float pid_result = 0.0;

unsigned long startedControlsTime;

long long counter = 0;


// INTERRUPTS ======================================================================

// 1600 Hz control ISR with reentrancy guard
static volatile bool control_running = false;
static volatile bool first_loop_with_data = true;
static volatile float initial_yaw = 0;

static float wrapAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static void controlISR() {
    // Skip this tick if control is still running from previous tick
    if (control_running) return;

    control_running = true;

    digitalWrite(VN_TRIGGER_PIN, HIGH);
    delayMicroseconds(VN_TRIGGER_PULSE_US);
    digitalWrite(VN_TRIGGER_PIN, LOW);
    IMUReadStatus read_status = readIMUFrameBlocking(IMU_FRAME_TIMEOUT_MS);

    // debugging
    // if (read_status == IMU_READ_CRC_FAIL) {
    //   imu_crc_fail_count++;
    //   return;
    // }
    // if (read_status == IMU_READ_TIMEOUT) {
    //   imu_timeout_count++;
    //   return;
    // }

    // imu_frame_ok_count++;

    // Run control if a new IMU frame has arrived
    if (getLatestIMUData(imu_data)) {
        Serial.println("entered IMU data parse step");
        double time_s = imu_data.timeUTC * 1e-3;

        // spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
        
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

        if(first_loop_with_data){
          first_loop_with_data = false;
          initial_yaw = roll_heading;
        }

        float heading = wrapAngle180(roll_heading-initial_yaw);

        // PID + feedforward control
        pid_result = PIDControl(heading, roll_velocity);

        float angular_accel = x_hat[2];
        float ff_result = feedForwardControl(angular_accel);
        // float ff_result = 0;
        float motor_cmd = pid_result + ff_result;


        // if (abs(roll_heading-initial_yaw) > 170) {
        //   // control_running = true;
        //   motor_cmd = 0;
        //   Serial.println("PID result: 1111111111111111111111");
        // }

        // if (!tuningHandshakeAlive()) {
        //   // Deadman watchdog: host heartbeat missing, force safe output.
        //   motor_cmd = 0.0f;
        // }

        Serial.print( "PID Output: "); Serial.println(pid_result, 3);
        Serial.print( "FF Output: "); Serial.println(ff_result, 5);
        Serial.print( "Orientation: "); Serial.println(heading);
        Serial.print( "Angular Accel: "); Serial.println(angular_accel);
        Serial.println(); Serial.println(); Serial.println();


        setTorque(min(motor_cmd, 1.5));
        // setTorque(0.0);

        // Tuning telemetry
        // tuningSendTelemetry(time_s, roll_heading, roll_velocity, x_hat[2], pid_result, ff_result, motor_cmd);
    }

    control_running = false;
}

void buzz() {
  digitalWrite(BUZZER_PIN, HIGH);

  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}


// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);

  setup_slave();
  Serial.println("set up teensy as slave");

  bool bat_sensor_setup = initBatSensor();
  Serial.println("set up battery sensor!");
  if (!bat_sensor_setup) {
    while (1) {
      delay(200);
    }
  }

  // Initialize Altimeter
  bool success_alti_setup = initAltimeter();
  readAltimeter(flight_data);
  Serial.println("set up Altimeter!");
  if (!success_alti_setup) {
    while (1) {
      delay(200);
    }
  }

  // Initialize serial IMU (VectorNav on Serial2)
  bool success_imu_setup = initIMU(115200);
  if (!success_imu_setup) {
    while (1) {
      delay(200);
    }
  }
  pinMode(VN_TRIGGER_PIN, OUTPUT);
  digitalWrite(VN_TRIGGER_PIN, LOW);

  Serial.println("set up serial IMU on Serial5!");

  // Initialize Kalman Filter
  float dt0 = 1/10.0f; // seconds
  // // FIXME read imu and store to
  float roll_heading0 = 0.0;
  float roll_velocity0 = 0.0;
  Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  kf.init(dt0, z0);

  // call buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  buzz();

  current_phase = LANDED;

  setupControls();

  // clear errors 
  sendCmd("w axis0.error 0");

  // tuningServiceUsbCommands();

  control_timer.begin(controlISR, CONTROL_ISR_PERIOD_US);  
  // Set to priority 0 (highest) to preempt all other interrupts
  control_timer.priority(0);
}

// LOOP ============================================================================
void loop() {
  // Serial.print("Active_error: "); Serial.println(queryODrive("r axis0.active_errors"));
  // sendCmd("u 0\n");
  // sendCmd("w axis0.error 0");

  static float base_altitude_m = flight_data.altitude_m;
  float altitude_agl_m = flight_data.altitude_m - base_altitude_m;
  
  // readBatSensor(battery_data);
  // Serial.print("Voltage (V): ");Serial.println(battery_data.voltage_v);
  // Serial.print("Current (mA): ");Serial.println(battery_data.current_ma);
  // Serial.print("Power (mW): ");Serial.println(battery_data.power_mw);
  // Serial.print("Load Voltage (V): ");Serial.println(battery_data.load_voltage_V);
  // Serial.println();

  // readAltimeter(flight_data);
  // Serial.print("Temp (C): ");Serial.println(flight_data.temp_C);
  // Serial.print("Pressure (hPa): ");Serial.println(flight_data.pressure_hPa);
  // Serial.print("Altitude (meters): ");Serial.println(altitude_agl_m);
  // Serial.println();

  // Serial.print("Time (s): "); Serial.println(imu_data.timeUTC, 10);

  // Serial.print( "YPR (deg): "); Serial.print(imu_data.ypr[0], 2); Serial.print(", "); Serial.print(imu_data.ypr[1], 2); Serial.print(", "); Serial.println(imu_data.ypr[2], 2);
  // Serial.print( "Angular Rate: "); Serial.print(imu_data.angularRate[0], 2); Serial.print(", "); Serial.print(imu_data.angularRate[1], 2); Serial.print(", "); Serial.println(imu_data.angularRate[2], 2);
  // Serial.print( "PID Output: "); Serial.println(pid_result);
  // Serial.print( "Longitude: "); Serial.print(imu_data.posLla[0]); Serial.println();
  // Serial.print( "Latitude: "); Serial.print(imu_data.posLla[1]); Serial.println();      

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

  // Serial.print("Current Phase: "); Serial.println(current_phase);

  // LAUNCH_PAD,
  // BOOST,
  // APOGEE,
  // RUN_CONTROLS,
  // LANDED

  // Serial.print("Altitude: "); Serial.println(altitude_agl_m);
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
      // setTorque(0.0);
      // setVel(); 
      break;


    //default:
      
      //Serial.println("Default");
      // Insert final data handling

  }

}
