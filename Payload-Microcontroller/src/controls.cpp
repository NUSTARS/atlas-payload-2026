// controls.cpp
//
// houses the control algorithm
#include "controls.h"


// Tuning values
// should all be positive
static float P = 0.0f;
static float I = 0.0f;
static float D = 10.0f;
static const float ISaturate = 10.0f;

static float FFGain = 0.0f;

unsigned long prevT = 0;
float prevError = 0;
float iTerm = 0;

void setPGain(float p) { P = p; }

void setIGain(float i) { I = i; }

void setDGain(float d) { D = d; }

void setFFGain(float ff) { FFGain = ff; }

float getPGain() { return P; }

float getIGain() { return I; }

float getDGain() { return D; }

float getFFGain() { return FFGain; }

void resetIntegralTerm() { iTerm = 0.0f; }


// PIDControl: float -> float
// converts an error into a control value
// using PID control
float PIDControl(float heading, float angularVel) {
    // If we want to integrate and/or
    // differentiate w.r.t. deltaT
    /*
    unsigned long t = millis();
    unsigned long deltaT = t - prevT;
    prevT = t;
    */
    
    // this will likely need filtering

    //float dError = error - prevError;

    iTerm += I * heading;
    iTerm = max(min(iTerm, ISaturate), -ISaturate);

    return (P * heading) - (D * angularVel) + iTerm;
}

float feedForwardControl(float angularAcc) {
  return FFGain * angularAcc;
}

// converts torques into analog pwm signals
void motorControl(float torque) {
    
    uint8_t voltage = (uint8_t) torque * 12345; // sus
    analogWrite(MOTOR_PWM_PIN, voltage);
}


void sendCmd(const char* cmd) {
  Serial2.print(cmd);
  Serial2.print('\n');
  delay(50);
}

String readLine(uint32_t timeout_ms = 200) {
  Serial2.setTimeout(timeout_ms);
  String s = Serial2.readStringUntil('\n');
  s.trim();
  return s;
}

String queryODrive(const char* cmd) {
  while (Serial2.available()) Serial2.read();
  Serial2.print(cmd);
  Serial2.print('\n');
  return readLine();
}


void setupControls() {
    Serial2.begin(115200);

    // Put axis in IDLE first
    sendCmd("w axis0.requested_state 1");
    delay(200);

    // Set velocity mode before requesting closed loop
    sendCmd("w axis0.controller.config.control_mode 1");
    delay(100);

    // Optional: set no torque
    sendCmd("w axis0.controller.input_torque 0.0");
    delay(100);

    // Now try to enter closed loop
    sendCmd("w axis0.requested_state 8");
    delay(500);

    // Print status
    // Serial.print("active_errors: ");
    // Serial.println(queryODrive("r axis0.active_errors"));

    // Serial.print("disarm_reason: ");
    // Serial.println(queryODrive("r axis0.disarm_reason"));

    // Serial.print("current_state: ");
    // Serial.println(queryODrive("r axis0.current_state"));

    // Serial.print("input_vel: ");
    // Serial.println(queryODrive("r axis0.controller.input_vel"));
}

void setTorque(float torque_val) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "w axis0.controller.input_torque %.3f", torque_val);
    sendCmd(cmd);

    // Serial.print("active_errors: ");
    // Serial.println(queryODrive("r axis0.active_errors"));
}