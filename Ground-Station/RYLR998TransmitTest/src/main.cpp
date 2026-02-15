#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial lora(2,3) // PINS RX, TX 

String lora_TX_address = "1";
String lora_RX_address = "2";

void setup() {

  Serial.begin(9600);
  lora.begin(9600);

  loraSerial.println("AT+ADDRESS=1" + lora_TX_address);
  delay(500);
  loraSerial.println("AT+NETWORKID=5");
  delay(500);
  loraSerial.println("AT+band=125000,K");
  delay(500)
}

void loop() {
  lora.println("AT+SEND=" + lora_RX_address + ",5,HI");
  delay(1000);
}