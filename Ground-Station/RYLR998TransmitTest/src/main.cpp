#include <Arduino.h>
#define LORA Serial1   // UART1

void sendCommand(const char *cmd) {
  LORA.println(cmd);
  delay(200);
  while (LORA.available()) {
    Serial.write(LORA.read());
  }
}

void setup() {
  Serial.begin(115200);
  LORA.begin(115200);  // default baud rate for RYLR998

  delay(2000);

  // Set address (must be unique)
  sendCommand("AT+ADDRESS=1");

  // Set network ID (must match receiver if using AT mode)
  sendCommand("AT+NETWORKID=5");

  // Set RF parameters:
  // Format:
  // AT+PARAMETER=<SF>,<BW>,<CR>,<Preamble>
  //
  // SF7
  // BW=125kHz (7)
  // CR=4/5 (1)
  // Preamble=8

  sendCommand("AT+PARAMETER=7,7,1,8");

  // Set frequency (example 915 MHz)
  sendCommand("AT+BAND=915000000");

  // Set output power (max 22 dBm)
  sendCommand("AT+CRFOP=15");

  Serial.println("LoRa Ready");
}

void loop() {

  uint8_t payload[24];

  for (int i = 0; i < 24; i++) {
    payload[i] = i;   // example data
  }

  sendCommand("AT+SEND=2,24,");

  delay(5000);
}
