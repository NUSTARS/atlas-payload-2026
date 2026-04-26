#include "tuning.h"

#include <Arduino.h>
#include <controls.h>
#include <stdlib.h>

#define TELEMETRY_DECIMATION 2
#define HEARTBEAT_TIMEOUT_MS 250

static String usb_rx_line;
static volatile uint32_t last_hb_ms = 0;
static volatile bool heartbeat_seen = false;

static void printGainSnapshot() {
  Serial.print("GAINS,");
  Serial.print(getPGain(), 6); Serial.print(",");
  Serial.print(getIGain(), 6); Serial.print(",");
  Serial.print(getDGain(), 6); Serial.print(",");
  Serial.println(getFFGain(), 6);
}

static void printHelp() {
  Serial.println("CMDS,HB|SET P <v>|SET I <v>|SET D <v>|SET FF <v>|GET GAINS|RESET I");
}

static void processUsbCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("HELP")) {
    printHelp();
    return;
  }

  if (line.equalsIgnoreCase("GET GAINS") || line.equalsIgnoreCase("GAINS")) {
    printGainSnapshot();
    return;
  }

  if (line.equalsIgnoreCase("HB")) {
    last_hb_ms = millis();
    heartbeat_seen = true;
    return;
  }

  if (line.equalsIgnoreCase("RESET I")) {
    resetIntegralTerm();
    Serial.println("ACK,RESET,I");
    return;
  }

  if (line.startsWith("SET ")) {
    int sp2 = line.indexOf(' ', 4);
    if (sp2 < 0) {
      Serial.println("ERR,BAD_SET_FORMAT");
      return;
    }

    String key = line.substring(4, sp2);
    key.trim();
    key.toUpperCase();

    String valueStr = line.substring(sp2 + 1);
    valueStr.trim();
    if (valueStr.length() == 0) {
      Serial.println("ERR,BAD_SET_VALUE");
      return;
    }

    char *endPtr = nullptr;
    float value = strtof(valueStr.c_str(), &endPtr);
    if (endPtr == valueStr.c_str()) {
      Serial.println("ERR,BAD_SET_VALUE");
      return;
    }

    if (key == "P") {
      setPGain(value);
    } else if (key == "I") {
      setIGain(value);
    } else if (key == "D") {
      setDGain(value);
    } else if (key == "FF") {
      setFFGain(value);
    } else {
      Serial.println("ERR,UNKNOWN_GAIN");
      return;
    }

    Serial.print("ACK,SET,");
    Serial.print(key);
    Serial.print(",");
    Serial.println(value, 6);
    return;
  }

  Serial.print("ERR,UNKNOWN_CMD,");
  Serial.println(line);
}

void tuningSetup() {
  printHelp();
  printGainSnapshot();
}

void tuningServiceUsbCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      processUsbCommand(usb_rx_line);
      usb_rx_line = "";
      continue;
    }

    if (usb_rx_line.length() < 120) {
      usb_rx_line += c;
    }
  }
}

bool tuningHandshakeAlive() {
  if (!heartbeat_seen) return false;
  uint32_t now = millis();
  return (uint32_t)(now - last_hb_ms) <= HEARTBEAT_TIMEOUT_MS;
}

void tuningSendTelemetry(
    double time_s,
    float roll_heading,
    float roll_velocity,
    float roll_accel_est,
    float pid_result,
    float ff_result,
    float motor_cmd
) {
  static uint8_t telem_div = 0;
  telem_div++;
  if (telem_div < TELEMETRY_DECIMATION) return;
  telem_div = 0;

  Serial.print("TEL,");
  Serial.print(time_s, 6); Serial.print(",");
  Serial.print(roll_heading, 6); Serial.print(",");
  Serial.print(roll_velocity, 6); Serial.print(",");
  Serial.print(roll_accel_est, 6); Serial.print(",");
  Serial.print(pid_result, 6); Serial.print(",");
  Serial.print(ff_result, 6); Serial.print(",");
  Serial.print(motor_cmd, 6); Serial.print(",");
  Serial.print(getPGain(), 6); Serial.print(",");
  Serial.print(getIGain(), 6); Serial.print(",");
  Serial.print(getDGain(), 6); Serial.print(",");
  Serial.println(getFFGain(), 6);
}