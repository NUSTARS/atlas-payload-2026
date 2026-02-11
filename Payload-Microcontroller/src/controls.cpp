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

const float FFGain = 0.0;

unsigned long prevT = 0;
float prevError = 0;
float iTerm = 0;


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
    
    float gain = 0.0;

    return gain * angularAcc;
}

// converts torques into analog pwm signals
void motorControl(float torque) {
    
    uint8_t voltage = (uint8_t) torque * 12345; // sus
    analogWrite(MOTOR_PWM_PIN, voltage);
}

