// controls.cpp
//
// houses the control algorithm
#include "controls.h"


// Tuning values
// should all be positive
const float P = 0.0;
const float I = 0.0;
const float D = 0.0;
const float ISaturate = 10.0;


unsigned long prevT = 0;
float prevError = 0;
float iTerm = 0;


// PIDControl: float -> float
// converts an error into a control value
// using PID control
float PIDControl(float error) {
    // If we want to integrate and/or
    // differentiate w.r.t. deltaT
    /*
    unsigned long t = millis();
    unsigned long deltaT = t - prevT;
    prevT = t;
    */

    // this will likely need filtering
    float dError = error - prevError;
    
    iTerm += I * error;
    iTerm = max(min(iTerm, ISaturate), -ISaturate);

    return (P * error) - (D * dError) + iTerm;
}

float feedForwardControl(float error) {
    return 0.0;
}

// converts torques into analog pwm signals
void motorControl(float torque) {
    
    uint8_t voltage = (uint8_t) torque * 12345; // sus
    analogWrite(MOTOR_PWM_PIN, voltage);
}

