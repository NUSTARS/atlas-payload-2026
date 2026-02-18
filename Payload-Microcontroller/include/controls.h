// controls.h
#pragma once

#include <Arduino.h>

#define MOTOR_PWM_PIN 2

// PIDControl: float -> float
// creates a control value using the PID algorithm, with
// the input heading used for the proportional error and in the
// computation of the integral error
// and the input angularVel being used for the derivative error
float PIDControl(float heading, float angularVel);

// feedForwardControl: float -> float
// creates a control value proportional to 
// the input angular acceleration
float feedForwardControl(float angularAcc);

// sends a PWM signal to the motor encoder
// to command the appropriate torque response
void motorControl(float torque);