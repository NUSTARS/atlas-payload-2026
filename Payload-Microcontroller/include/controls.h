// controls.h
#pragma once

#include <Arduino.h>

#define MOTOR_PWM_PIN 2

// PIDControl: float -> float
// creates a control value (torque) using the PID algorithm, with
// the input heading used for the proportional error and in the
// computation of the integral error
// and the input angularVel being used for the derivative error
float PIDControl(float heading, float angularVel);

// feedForwardControl: float -> float
// creates a control value (torque) proportional to 
// the input angular acceleration
float feedForwardControl(float angularAcc);

// sends a PWM signal to the motor encoder
// to command the appropriate torque response
void motorControl(float torque);

// Runtime tuning hooks for live updates over USB serial.
void setPGain(float p);
void setIGain(float i);
void setDGain(float d);
void setFFGain(float ff);
float getPGain();
float getIGain();
float getDGain();
float getFFGain();
void resetIntegralTerm();


// TEST UART 
void sendCmd(const char* cmd);

String readLine(uint32_t timeout_ms = 200);

String queryODrive(const char* cmd);

void setupControls();

void setTorque(float torque_val);

void setVel();