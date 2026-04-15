// tuning.h
#pragma once
#include <Arduino.h>

// Initializes tuning I/O over USB and prints command help/current gains.
void tuningSetup();

// Processes inbound tuning commands from the USB serial interface.
void tuningServiceUsbCommands();

// Returns true when heartbeat packets are arriving within timeout.
// Safe to call from the control ISR.
bool tuningHandshakeAlive();

// Sends decimated control telemetry as CSV-formatted serial output.
void tuningSendTelemetry(
	double time_s,
	float roll_heading,
	float roll_velocity,
	float roll_accel_est,
	float pid_result,
	float ff_result,
	float motor_cmd
);