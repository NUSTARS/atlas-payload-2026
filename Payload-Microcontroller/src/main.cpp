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
#define VN_TRIGGER_PIN 19
#define VN_TRIGGER_PULSE_US 50
#define IMU_FRAME_TIMEOUT_MS 100
#define CONTROL_ISR_PERIOD_US 1000000

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
      motorControl(0.0f);
      return;
    }
    if (read_status == IMU_READ_TIMEOUT) {
      imu_timeout_count++;
      motorControl(0.0f);
      return;
    }

    imu_frame_ok_count++;

    if (current_phase != RUN_CONTROLS) {
      return;
    }

    if (getLatestIMUData(imu_data)) {
        double time_s = imu_data.timeUTC * 1e-3;

        spi_packet_set_imu(imu_data.ypr, imu_data.angularRate, imu_data.posLla);
        
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
        return;
    }

    motorControl(0.0f);
}

// SETUP ===========================================================================
void setup() {
  Serial.begin(115200);
  // delay(1500);

  setup_slave();
  Serial.println("set up teensy as slave");

  setupControls();

  // Initialize serial IMU (VectorNav on Serial5)
  initIMU();
  Serial.println("set up serial IMU on Serial5!");

  pinMode(VN_TRIGGER_PIN, OUTPUT);
  digitalWrite(VN_TRIGGER_PIN, LOW);

  // Initialize Kalman Filter
  float dt0 = 1/100.0f; // seconds
  // // FIXME read imu and store to
  float roll_heading0 = 0.0;
  float roll_velocity0 = 0.0;
  Eigen::Vector<float,2> z0 = meas_vector(roll_heading0, roll_velocity0);
  kf.init(dt0, z0);

  // Initialize 1 Hz SyncIn trigger + control ISR
  control_timer.begin(controlISR, CONTROL_ISR_PERIOD_US);
  control_timer.priority(0);

}

// LOOP ============================================================================
void loop() {
  tuningServiceUsbCommands();

  static uint32_t prev_debug_ms = 0;
  if (millis() - prev_debug_ms >= 1000) {
    prev_debug_ms = millis();
    Serial.print("IMU trig="); Serial.print(imu_trigger_count);
    Serial.print(" ok="); Serial.print(imu_frame_ok_count);
    Serial.print(" crc="); Serial.print(imu_crc_fail_count);
    Serial.print(" timeout="); Serial.println(imu_timeout_count);
  }
}