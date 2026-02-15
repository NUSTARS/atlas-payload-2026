// controls.h
#pragma once

#include <Arduino.h>

#define MOTOR_PWM_PIN 2

// PIDControl: float -> float
// converts an error into a control value
// using PID control
float PIDControl(float heading, float angularVel);

// feedForwardControl: float -> float
float feedForwardControl(float angularAcc);

// sends a PWM signal to the motor encoder
void motorControl(float torque);