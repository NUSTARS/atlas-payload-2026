
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoEigen.h>
#include <KalmanFilter.h>
#include <Adafruit_BNO08x.h>

#include <bno-imu.hpp>

/*
Kalman Filter to do list (order of priority)
 - Make sure the code actual runs. It should hopefully, fingers crossed, but there may be bugs
 - Experiment with both different filters and see how they do
 - Implement much of this code (whatever applies to our IMU) into main control loop.
 - Find exact values for var_omega and var_phi based on VN200 datasheet (again)
 - Add in the control input to KalmanFilter::predict(u) (this should take like 2 mins)
 - Hope all of this wasn't for nothing
*/



struct MeasTuple {
  float t;
  float phi;
  float omega;
};

// this is roll for BNO08x — ypr[2]=roll, angularRate[0]=roll rate
static constexpr uint8_t ORIENT_AXIS = 2;
static constexpr uint8_t GYRO_AXIS   = 0;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

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
  readIMU(imu_data);
  const uint32_t t0_us = micros();

  delay(BNO055_SAMPLERATE_DELAY_MS);

  const uint32_t t1_us = micros();
  float dt0 = (t1_us - t0_us) * 1e-6f; // seconds
  Serial.println(dt0,7);
  float phi0  = imu_data.ypr[ORIENT_AXIS];
  float omega0 = imu_data.angularRate[GYRO_AXIS];

  Eigen::Vector<float,2> z0 = meas_vector(phi0, omega0);
  kf.init(dt0, z0);

  Serial.println("KF initialized");
}

void loop()
{
  uint8_t got = readIMU(imu_data);

  // Only run the KF when we have a fresh rotation vector — without it ypr is
  // stale from the previous call and would feed bad data into the filter
  if ((got & (IMU_GOT_ROTVEC | IMU_GOT_GYRO)) != (IMU_GOT_ROTVEC | IMU_GOT_GYRO)) return;

  float roll_heading = imu_data.ypr[ORIENT_AXIS];
  float roll_velocity = imu_data.angularRate[GYRO_AXIS];
  // Serial.print("Meas:\t");
  // Serial.print(roll_heading, 6); Serial.print("\t"); Serial.println(roll_velocity, 6);
  // Kalman filter
  // Eigen::Vector3f x_hat = kf.state();
  // Serial.print("xhat:\t");
  // Serial.print(x_hat(0), 6); Serial.print("\t");
  // Serial.print(x_hat(1), 6); Serial.print("\t");

  Eigen::Vector<float,2> z = meas_vector(roll_heading, roll_velocity);
  Serial.print("Meas:\t");
  Serial.print(z(0), 6); Serial.print("\t"); Serial.println(z(1), 6);

  uint32_t ts_us = micros();
  kf.predict();

  Eigen::Vector3f x_hat1 = kf.state();
  Serial.print("predx:\t");
  Serial.print(x_hat1(0), 6); Serial.print("\t");
  Serial.println(x_hat1(1), 6);// Serial.print("\t");

  kf.update(z);
  uint32_t tf_us = micros();

  Eigen::Vector3f x_hat2 = kf.state();
  Serial.print("updx:\t");
  Serial.print(x_hat2(0), 6); Serial.print("\t");
  Serial.print(x_hat2(1), 6); Serial.print("\t");
  Serial.println(x_hat2(2), 6);

  auto y = kf.innovation();
  Serial.print("y:  \t");
  Serial.print(y(0), 6); Serial.print("\t");
  Serial.println(y(1), 6);
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

  Serial.print("Kalman Time (us): "); Serial.println(tf_us - ts_us); 
  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

