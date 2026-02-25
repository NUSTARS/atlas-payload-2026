

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoEigen.h>

#include <KalmanFilterWN.h>
#include <KalmanFilterGM.h>


/*
Kalman Filter to do list (order of priority)
 - Make sure the code actual runs. It should hopefully, fingers crossed, but there may be bugs
 - Experiment with both different filters and see how they do
 - Implement much of this code (whatever applies to our IMU) into main control loop. 
 - Find exact values for var_omega and var_phi based on VN200 datasheet (again)
 - Add in the control input to KalmanFilter::predict(u) (this should take like 2 mins)
 - Hope all of this wasn't for nothing
*/


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void printEvent(sensors_event_t* event);

struct MeasTuple {
  float t;
  float phi;
  float omega;
};
static MeasTuple meas;

// angle conversion
static constexpr bool CONVERT_TO_RADIANS = false;
static inline float deg2rad(float d) { return d * 0.017453292519943295f; }
static inline float rad2deg(float d) { return d * 57.2957795131f; }

// this is roll for BNO055 I believe, clearly not the same axis names
static constexpr uint8_t ORIENT_AXIS = 2; // 2=z is roll
static constexpr uint8_t GYRO_AXIS = 0; // 0=x is roll
static inline float getOrientComponent(const sensors_event_t& e, uint8_t axis) {
  if (axis == 0) return e.orientation.x;
  if (axis == 1) return e.orientation.y;
  return e.orientation.z;
}
static inline float getGyroComponent(const sensors_event_t& e, uint8_t axis) {
  if (axis == 0) return e.gyro.x;
  if (axis == 1) return e.gyro.y;
  return e.gyro.z;
}

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Control which Kalman Filter to use (0: White-Noise, 1: Gauss-Markov). Try both and see how they do.
#define USE_GM_KALMANFILTER 0

#if USE_GM_KALMANFILTER
  KalmanFilterGM kf;
#else
  KalmanFilterWN kf;
#endif

void setup(void)
{
  // get started
  Serial.begin(115200);
  while (!Serial) delay(10); 
  Serial.println("Orientation Sensor Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("BNO055 not detected");
    while (1);
  }
  
  delay(1000);
  sensors_event_t o0, g0, o1, g1;

  // first sample
  bno.getEvent(&o0, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&g0, Adafruit_BNO055::VECTOR_GYROSCOPE);
  const uint32_t t0_us = micros();

  delay(BNO055_SAMPLERATE_DELAY_MS); // small spacing to get dt0

  // second sample just time
  const uint32_t t1_us = micros();

  float dt0 = (t1_us - t0_us) * 1e-6f; // secs (lol)
  float phi0 = getOrientComponent(o0, ORIENT_AXIS); // degrees
  float omega0 = getGyroComponent(g0, GYRO_AXIS); // radians

  // IMUs give roll and heading in different degree units for some reason
  // I also found the angular rate and angle to be reversed sign
  if (CONVERT_TO_RADIANS) {
    phi0 = deg2rad(phi0);
    omega0 = -omega0;
  } else {
    phi0 = phi0; 
    omega0 = -rad2deg(omega0);
  }  
  // iniital states 
  Eigen::Matrix<float,3,3> P0;
  P0 << 0.1, 0, 0,
        0, 0.1, 0,
        0, 0, 100; // acc has higher covariance
  Eigen::Matrix<float,3,1> x0;
  x0 << phi0, omega0, 0.0f;

  // values to tune
  /*
  if you use IMU specs to set var_phi and var_omega, be careful to get units right.
  This Kalman filter was made and tuned and tested with the VN200 IMU which has built-in 
  Kalman filtering for omega and phi (but not alpha) so they are very accurate.
  BNO055 does not have this handy dandy feature meaning the tuning for this IMU will look
  different and won't give as good of results.
  */
  float var_phi = 1e-3f;
  float var_omega = 1e-3f;

  #if USE_GM_KALMANFILTER
    // you only need to tune tau_corr and sig_tau here (moment of inertia too but we know that)
    float tau_corr = 0.01f; // seconds
    float sig_tau = 0.1f; // N*m, represents the noise in torque

    float Inertia = 0.05f; // this is a random number idk the breadboard or rocket moments of inertia
    float sig_alpha = sig_tau / Inertia;
    float var_alpha = sig_alpha*sig_alpha;
    
    // initialize kalman with parameters
    kf.init(var_alpha, var_phi, var_omega, tau_corr, dt0, x0, P0);
  #else
    // var process controls the white noise. Go to KalmanFilterWNN::update_timestep() for a ton more information
    float var_process = 1e1f;
    // the first three are the most important. Feel free to set these to 0 and 1 respectively for now or leave as is
    float damping = -0.1f; 
    float accel_passthrough = 0.95f;
    // initialize kalman with parameters
    kf.init(var_process, var_phi, var_omega, damping, accel_passthrough, dt0, x0, P0);
  #endif
  
  Serial.println("ready");
  Serial.print("Meas:\t");
  Serial.print(phi0, 6);   Serial.print("\t");
  Serial.print(omega0, 6); Serial.print("\t");
  Serial.print("Estimated:\tphi:");
  auto xhat = kf.state();
  Serial.print(xhat(0), 6); Serial.print("\tomega");
  Serial.print(xhat(1), 6); Serial.print("\talpha\n");
  auto p = kf.covariance();
  Serial.print(p(0,0));Serial.print(p(1,1));Serial.print(p(2,2));
  Serial.print("\n");
  delay(6000);
}

void loop()
{
  meas.t = 1e-6f * (float)micros(); // set time 
  
  // grab heading and roll
  sensors_event_t orientationData, angVelocityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  float phi_raw = getOrientComponent(orientationData, ORIENT_AXIS);
  float omega_raw = getGyroComponent(angVelocityData, GYRO_AXIS);

  // same important angle conversion as before
  if (CONVERT_TO_RADIANS) {
    meas.phi = deg2rad(phi_raw);
    meas.omega = -omega_raw;
  } else {
    meas.phi = phi_raw;
    meas.omega = -rad2deg(omega_raw);
  }  
  // turn measurement z into a vector here 
  Eigen::Vector<float,2> z;
  z << meas.phi, meas.omega; 

  // kalman predict and update phases
  kf.predict(); // we'll eventually give this the control input torque, u. 
  kf.update(z);

  auto xhat = kf.state();

  // this might be printed kind of ugly, I changed some formatting
  // order is t, phi, omega, alpha. Compare by column
  Serial.print("Meas:\t");
  Serial.print(meas.t, 6);   Serial.print("\t");
  Serial.print(meas.phi, 6); Serial.print("\t");
  Serial.println(meas.omega, 6); // no measured acceleration 
  Serial.print("xhat:\t");
  Serial.print(meas.t, 6);   Serial.print("\t");
  Serial.print(xhat(0), 6); Serial.print("\t");
  Serial.print(xhat(1), 6); Serial.print("\t");
  Serial.println(xhat(2), 6); // and acceleration
  // can be difficult to read but possible to get <1 degree error
  Serial.print("Error:\t");
  Serial.print(meas.t, 6);   Serial.print("\t");
  Serial.print(xhat(0) - meas.phi, 6); Serial.print("\t");
  Serial.print(xhat(1) - meas.omega, 6); Serial.print("\t\n");
  Serial.println("--");

  delay(BNO055_SAMPLERATE_DELAY_MS); // delay to next timestep
}




// helper to print a matrix for debugging; 
template<typename MatType>
void printMatrix(const MatType& M, const char* name = "")
{
    if (name && name[0] != '\0') {
        Serial.println(name);
    }
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            Serial.print(M(i,j), 6);   // 6 decimal places
            Serial.print("  ");
        }
        Serial.println();
    }
    Serial.println();
}



// this is a function for printing measurements which you can completely ignore
void printEvent(sensors_event_t* event) {
  float x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/
