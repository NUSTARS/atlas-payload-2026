// How to read error from motor using UART:

/*
1. Output is number
2. Convert number to binary
3. Bit field determines which error

Example:
1. Output is 64
2. Binary is 0b1000000
3. Bit field is 6 (starts at 0, from LSB)

Common errors:
0	INVALID_STATE (0)
1	MOTOR_FAILED (1)
6	WATCHDOG_TIMER_EXPIRED (64)
8	ESTOP_REQUESTED (256)
*/

String readODrive(String cmd)
{
  Serial1.print(cmd);
  Serial1.print("\n");

  String resp = "";
  unsigned long t = millis();

  while (millis() - t < 50) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') return resp;
      resp += c;
    }
  }
  return resp;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {

  String axisErr = readODrive("r odrv0.axis0.error");

  Serial.print("Axis error: ");
  Serial.println(axisErr);

  delay(500);
}