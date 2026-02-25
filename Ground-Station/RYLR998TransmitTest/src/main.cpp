#include <Arduino.h>
#define LORA Serial1   // UART1
uint8_t numValues = 12;
#define payloadSize (numValues * 2)

void sendCommand(const char *cmd) {
  LORA.println(cmd);
  delay(200);
  while (LORA.available()) {
    Serial.write(LORA.read());
  }
}

void sendLoRa(int16_t *data, int numValues) {

  int totalBytes = numValues * 2;

  LORA.print("AT+SEND=2,");
  LORA.print(totalBytes * 2);   // hex string length
  LORA.print(",");

  for (int i = 0; i < numValues; i++) {

    int16_t value = data[i];

    uint8_t lowByte  = value & 0xFF;          // LSB first
    uint8_t highByte = (value >> 8) & 0xFF;   // MSB second

    if (lowByte < 16) LORA.print("0");
    LORA.print(lowByte, HEX);

    if (highByte < 16) LORA.print("0");
    LORA.print(highByte, HEX);
  }

  LORA.println();

  delay(200);

  while (LORA.available()) {
    uint8_t b = LORA.read();
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

uint8_t frameNumber = 0;

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

  int16_t dummy_frames[5][numValues] = {
      {1000,  360, 130, -100,  7500, 14000,   0,  0,   0,  0, 100, 0},
      {2000,  125, 135,  140,  7501, 14001,   5, -10, 10,  1, 90, 1},
      {4000,  130, 10,  160,  7600, 14200,  10, 200, 30,  1, 40, 2},
      {8000,  140, 100,  180,  7700, 14300,  15, 400, 50,  2, 20, 3},
      {12000, 135, 140,  170,  7800, 14400,   8, -100, 25,  2, 10, 4}
  };

  sendLoRa(dummy_frames[frameNumber], numValues);
  frameNumber = (frameNumber + 1) % 5;  // cycle through frames
  delay(1000);
}
