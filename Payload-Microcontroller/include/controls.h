// controls.h
#pragma once

#include <Arduino.h>

#define MOTOR_PWM_PIN 2

// PIDControl: float -> float
// converts an error into a control value
// using PID control
float PIDControl(float error);

// feedForwardControl: float -> float
float feedForwardControl(float error);

void motorControl(float torque);