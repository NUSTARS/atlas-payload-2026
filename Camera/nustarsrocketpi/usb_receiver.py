# IDK WHAT THIS IS FOR - NATHAN WU

import serial, struct, time

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
time.sleep(1.5)
ser.reset_input_buffer()

i = 0
buf = bytearray()

while True:
    buf += ser.read(ser.in_waiting or 1)   # grab whatever is available
    while len(buf) >= 4:
        x = struct.unpack_from("<I", buf, 0)[0]
        del buf[:4]
        print(f"{i}: {x}")
        i += 1
