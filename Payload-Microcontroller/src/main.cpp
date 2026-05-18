#include <Arduino.h>

#include <altimeter.h>
#include <bat-sensor.h>
#include <controls.h>
#include <pi-communication.h>
#include <KalmanFilter.h>
#include <imu.h>
// #include <tuning.h>

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
#define VN_TRIGGER_PIN 41
#define VN_TRIGGER_PULSE_US 5
#define IMU_FRAME_TIMEOUT_MS 10
#define CONTROL_ISR_PERIOD_US 100000

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

static volatile uint32_t imu_trigger_count = 0;
static volatile uint32_t imu_frame_ok_count = 0;
static volatile uint32_t imu_crc_fail_count = 0;
static volatile uint32_t imu_timeout_count = 0;

static void controlISR() {
    imu_trigger_count++;

    digitalWrite(VN_TRIGGER_PIN, HIGH);
    delayMicroseconds(VN_TRIGGER_PULSE_US);
    digitalWrite(VN_TRIGGER_PIN, LOW);
    IMUReadStatus read_status = readIMUFrameBlocking(IMU_FRAME_TIMEOUT_MS);
    if (read_status == IMU_READ_CRC_FAIL) {
      imu_crc_fail_count++;
      return;
    }
    if (read_status == IMU_READ_TIMEOUT) {
      imu_timeout_count++;
      return;
    }

    imu_frame_ok_count++;
    getLatestIMUData(imu_data);
    
}

// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);  // give serial time to initialize
  Serial.println("DEBUG: setup() started");

  setup_slave();
  Serial.println("DEBUG: set up teensy as slave");

  setupControls();
  Serial.println("DEBUG: setupControls() done");

  // Initialize serial IMU (VectorNav on Serial5)
  initIMU();
  Serial.println("DEBUG: set up serial IMU on Serial5!");

  pinMode(VN_TRIGGER_PIN, OUTPUT);
  digitalWrite(VN_TRIGGER_PIN, LOW);
  Serial.println("DEBUG: pin 19 initialized");

  // Initialize Kalman Filter
  float dt0 = 1/100.0f; // seconds
  // // FIXME read imu and store to
  float roll_heading0 = 0.0;
  float roll_velocity0 = 0.0;
  Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  kf.init(dt0, z0);
  Serial.println("DEBUG: Kalman filter initialized");

  // Initialize 1 Hz SyncIn trigger + control ISR
  Serial.println("DEBUG: about to start timer");
  control_timer.begin(controlISR, CONTROL_ISR_PERIOD_US);
  control_timer.priority(0);
  Serial.println("DEBUG: timer started");

}

// LOOP ============================================================================
void loop() {
  getLatestIMUData(imu_data);
  Serial.println(imu_data.timeUTC, 3);
  Serial.print(imu_data.ypr[0]);
  Serial.print(", ");
  Serial.print(imu_data.ypr[1]);
  Serial.print(", ");
  Serial.print(imu_data.ypr[2]);
  Serial.println();
  Serial.print(imu_data.angularRate[0]);
  Serial.print(", ");
  Serial.print(imu_data.angularRate[1]);
  Serial.print(", ");
  Serial.print(imu_data.angularRate[2]);
  Serial.println();
  Serial.print(imu_data.accel[0]);
  Serial.print(", ");
  Serial.print(imu_data.accel[1]);
  Serial.print(", ");
  Serial.print(imu_data.accel[2]);
  Serial.println();
  delay(10);
}