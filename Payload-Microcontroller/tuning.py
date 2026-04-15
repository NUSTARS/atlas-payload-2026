# tuning.py
# sends gains as commands over serial
# Format: P I D ISaturate FF
import serial

spString = input("enter serial port: ")
ser = serial.Serial(spString, 9600, timeout=1);
while True:
    try:
        gains = list(map(lambda x: float(x), input("P I D ISaturate FF: ").split(" ")))
        if len(gains) == 5:
            print(gains)
            byteGains = bytes(gains)
            ser.write(byteGains)

    except KeyboardInterrupt:
        break
    except:
        continue