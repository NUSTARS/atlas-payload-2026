
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoEigen.h>
#include <KalmanFilter.h>
#include <imu.h>


struct MeasTuple {
  float t;
  float phi;
  float omega;
};

// this is roll for BNO08x — ypr[2]=roll, angularRate[0]=roll rate
static constexpr uint8_t ORIENT_AXIS = 2;
static constexpr uint8_t GYRO_AXIS = 0;

/* Set the delay between fresh samples */
uint16_t SAMPLERATE_DELAY_MS = 10;

KalmanFilter kf;
IMUData imu_data;

// Must be defined before loop() — template functions are not visible if defined after their call site
template<typename MatType>
void printMatrix(const MatType& M, const char* name = "")
{
    if (name && name[0] != '\0') {
        Serial.println(name);
    }
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            Serial.print(M(i,j), 8);
            Serial.print("  ");
        }
        Serial.println();
    }
    Serial.println();
}

void setup(void)
{
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("KF test starting...");

  bool ok = initIMU();
  if (!ok) {
    Serial.println("IMU init failed — halting");
    while (1) delay(100);
  }

  delay(1000);
  
  // first sample to seed the KF initial state
  // serviceIMU();
  // static uint32_t last_seq = 0;
  // uint32_t now_seq = getIMUSequence();
  // if (getLatestIMUData(imu_data)) {
  //   // && current_phase == RUN_CONTROLS) {
  //   // debug printout of time and YPR of newest IMU packet
  //  //  double time_s1 = imu_data.timeUTC * 1e-3;

  // Make sure this is correct
  float dt0 = 0.01; // (t1_us - t0_us) * 1e-6f; // seconds
  
  float phi0 = imu_data.ypr[ORIENT_AXIS];
  float omega0 = imu_data.angularRate[GYRO_AXIS];

  Eigen::Vector<float,2> z0 = {phi0, omega0};//meas_vector(phi0, omega0);
  kf.init(dt0, z0);

  Serial.println("KF initialized");
}

void loop() {
  // Run control immediately when a new IMU frame is published.
  serviceIMU();

  // Run control immediately when a new IMU frame is published.
  static uint32_t last_seq = 0;
  uint32_t now_seq = getIMUSequence();
  // Serial.println('a');
  if (now_seq != last_seq) {
    last_seq = now_seq;
    if (getLatestIMUData(imu_data)) {
      // double time_s = imu_data.timeUTC * 1e-3;
      // Serial.print("Time: "); Serial.println(time_s, 3);
      float roll_heading = imu_data.ypr[2];
      float roll_velocity = imu_data.angularRate[0];
      Serial.print("Meas:\t");
      Serial.print(roll_heading, 6); Serial.print("\t"); Serial.println(roll_velocity, 6);
      // Kalman filter
      Eigen::Vector<float,2> z = {roll_heading, roll_velocity};
      kf.predict();
      kf.update(z);
      Eigen::Vector3f x_hat = kf.state();
      Serial.print("xhat:\t");
      Serial.print(x_hat(0), 6); Serial.print("\t");
      Serial.print(x_hat(1), 6); Serial.print("\t");
      Serial.println(x_hat(2), 6);
      auto y = kf.innovation();
      Serial.print("y:  \t");
      Serial.print(y(0), 6); Serial.print("\t");
      Serial.println(y(1), 6);
      Serial.println(x_hat(2), 6);
      // auto K = kf.getK();
      // printMatrix(K, "K:");
      // auto F = kf.getF();
      // printMatrix(F, "F:");
      // auto P = kf.covariance();
      // printMatrix(P, "P:");
      // auto Q = kf.getQ();
      // printMatrix(Q, "Q:");

      // Serial.print("YPR: ");
      // Serial.print(imu_data.ypr[0], 4); Serial.print(", ");
      // Serial.print(imu_data.ypr[1], 4); Serial.print(", ");
      // Serial.println(imu_data.ypr[2], 4);
      Serial.println("--");
    }
  }
  // delay(SAMPLERATE_DELAY_MS);
  // getLatestIMUData(imu_data);
// debug printout of time and YPR of newest IMU packet

 // uint8_t got = readIMU(imu_data);




  // Only run the KF when we have a fresh rotation vector — without it ypr is
  // stale from the previous call and would feed bad data into the filter
  ///if ((got & (IMU_GOT_ROTVEC | IMU_GOT_GYRO)) != (IMU_GOT_ROTVEC | IMU_GOT_GYRO)) return;

  // float roll_heading = imu_data.ypr[ORIENT_AXIS];
  // float roll_velocity = imu_data.angularRate[GYRO_AXIS];
  // Serial.print("Meas:\t");
  // Serial.print(roll_heading, 6); Serial.print("\t"); Serial.println(roll_velocity, 6);
  // Kalman filter
  // Eigen::Vector3f x_hat = kf.state();
  // Serial.print("xhat:\t");
  // Serial.print(x_hat(0), 6); Serial.print("\t");
  // Serial.print(x_hat(1), 6); Serial.print("\t");

  // Eigen::Vector<float,2> z = meas_vector(roll_heading, roll_velocity);
  // Serial.print("Meas:\t");
  // Serial.print(z(0), 6); Serial.print("\t"); Serial.println(z(1), 6);

  // uint32_t ts_us = micros();
  // kf.predict();

  // Eigen::Vector3f x_hat1 = kf.state();
  // Serial.print("predx:\t");
  // Serial.print(x_hat1(0), 6); Serial.print("\t");
  // Serial.println(x_hat1(1), 6);// Serial.print("\t");

  // kf.update(z);
  // uint32_t tf_us = micros();

  // Eigen::Vector3f x_hat2 = kf.state();
  // Serial.print("updx:\t");
  // Serial.print(x_hat2(0), 6); Serial.print("\t");
  // Serial.print(x_hat2(1), 6); Serial.print("\t");
  // Serial.println(x_hat2(2), 6);

  // auto y = kf.innovation();
  // Serial.print("y:  \t");
  // Serial.print(y(0), 6); Serial.print("\t");
  // Serial.println(y(1), 6);
  // Serial.print(z(0) - x_hat2(0), 6); Serial.print("\t");
  // Serial.print(y(0), 6); Serial.print("\t");
  // Serial.println(y(1), 6);
  // Serial.println(z(0) - x_hat(0), 6);
  // Serial.println(z(1) - x_hat2(1), 6);// Serial.print("\t");


  // Serial.println(x_hat(2), 6);
  // auto K = kf.getK();
  // printMatrix(K, "K:");
  // auto F = kf.getF();
  // printMatrix(F, "F:");
  // auto P = kf.covariance();
  // printMatrix(P, "P:");
  // auto Q = kf.getQ();
  // printMatrix(Q, "Q:");

  //Serial.print("Kalman Time (us): "); Serial.println(tf_us - ts_us); 
  // Serial.println("--");
  // delay(SAMPLERATE_DELAY_MS);
}

