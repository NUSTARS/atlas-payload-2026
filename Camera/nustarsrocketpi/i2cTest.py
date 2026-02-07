# JUST TESTING TO READ I2C FROM A BMP SENSOR

import smbus2
import time

# address of bno005
DEVICE_ADDRESS = 0x28
# The I2C bus number 
I2C_BUS = 1 

# Create an SMBus object
bus = smbus2.SMBus(I2C_BUS)

time.sleep(1)


chip_id = bus.read_byte_data(DEVICE_ADDRESS, 0x00)
assert chip_id == 0xA0


bus.write_byte_data(DEVICE_ADDRESS, 0x07, 0x00)
time.sleep(0.01)


bus.write_byte_data(DEVICE_ADDRESS, 0x3D, 0x00)   # CONFIG
time.sleep(0.03)

bus.write_byte_data(DEVICE_ADDRESS, 0x3D, 0x0C)   # NDOF (or IMU)
time.sleep(0.02)


try:
    while True:
        lsb = bus.read_byte_data(DEVICE_ADDRESS, 0x08)
        msb = bus.read_byte_data(DEVICE_ADDRESS, 0x09)

        accel_x = (msb << 8) | lsb
        if accel_x > 32767:
            accel_x -= 65536

        print(accel_x)
        time.sleep(0.1)
        

except KeyboardInterrupt:
    pass
finally:
    bus.close()
