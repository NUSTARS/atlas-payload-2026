// controls.cpp
//
// houses the control algorithm
#include "controls.h"


// Tuning values
// should all be positive

static float antiWindup = 30;

// best so far:
// static float P = 0.00060f;
// static float I = 0.00001f;
// static float D = 0.015f;

static float P = 0.00060f;
//0.00003f -> old I
static float I = 0.00001f;
// static float D = 0.0120f;
static float D = 0.015f;

static const float ISaturate = 0.0003f;

// static float FFGain = -0.0000004f;
static float FFGain = 0.000001f;

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

    iTerm += I * max(min(heading, antiWindup), -antiWindup);
    iTerm = max(min(iTerm, ISaturate), -ISaturate);

    return (P * heading) - (D * angularVel) + iTerm;
}

float feedForwardControl(float angularAcc) {
  return FFGain * angularAcc;
}

// converts torques into analog pwm signals
// void motorControl(float torque) {
    
//     // uint8_t voltage = (uint8_t) torque * 12345; // sus
//     // analogWrite(MOTOR_PWM_PIN, voltage);
//     setTorque(float torque_val)
// }


void sendCmd(const char* cmd) {
  Serial2.print(cmd);
  Serial2.print('\n');
  // delay(50);
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


// void setupControls() {
//     Serial2.begin(115200);

//     // Put axis in IDLE first
//     sendCmd("w axis0.requested_state 1");
//     delay(200);

//     // // Set torque mode before requesting closed loop
//     // sendCmd("w axis0.controller.config.control_mode 1");
//     // delay(100);

//     // Set torque mode before requesting closed loop
//     sendCmd("w axis0.controller.config.control_mode 2");
//     delay(500);
    
//     sendCmd("w axis0.controller.config.input_mode 1");
//     delay(500);

//     // Optional: set no torque
//     // sendCmd("w axis0.controller.input_torque 0.0");
//     // delay(100);

//     // Now try to enter closed loop
//     sendCmd("w axis0.requested_state 8");
//     delay(500);

//     sendCmd("w axis0.controller.input_vel 0.2");
//     delay(500);

//     // Print status
//     // Serial.print("active_errors: ");
//     // Serial.println(queryODrive("r axis0.active_errors"));

//     // Serial.print("disarm_reason: ");
//     // Serial.println(queryODrive("r axis0.disarm_reason"));

//     // Serial.print("current_state: ");
//     // Serial.println(queryODrive("r axis0.current_state"));

//     // Serial.print("input_vel: ");
//     // Serial.println(queryODrive("r axis0.controller.input_vel"));
// }

void setupControls() {
    Serial2.begin(115200);

    // Put axis in IDLE first
    sendCmd("w axis0.requested_state 1");   // IDLE
    delay(200);

    // Set control mode to TORQUE_CONTROL
    sendCmd("w axis0.controller.config.control_mode 1");
    delay(100);

    // Keep passthrough input mode
    sendCmd("w axis0.controller.config.input_mode 1");
    delay(100);

    // Enter closed loop control
    sendCmd("w axis0.requested_state 8");
    delay(500);

    // Serial.println(queryODrive("r axis0.current_state")); // 1
    // Serial.println(queryODrive("r axis0.active_errors")); // 0
    // Serial.println(queryODrive("r axis0.disarm_reason")); // 0
    // Serial.println(queryODrive("r axis0.procedure_result")); // 14

    delay(5000);

    sendCmd("w axis0.controller.input_torque 0");
    delay(100);
}

void setVel() {
    sendCmd("w axis0.controller.input_vel 1");

    // Serial.print("active_errors: ");
    // Serial.println(queryODrive("r axis0.active_errors"));

    // Serial.print("current state: ");
    // Serial.println(queryODrive("r axis0.current_state"));
}

void setTorque(float torque_val) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "w axis0.controller.input_torque %.3f", torque_val);
    sendCmd(cmd);

    // Serial.print("active_errors: ");
    // Serial.println(queryODrive("r axis0.active_errors"));
}