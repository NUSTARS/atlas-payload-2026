import loralibPi5 as loralib
import time
import struct
from dataclasses import dataclass

fq = 915000000 # 915 MHz
bw = 125 # 125 kHz
cr = 1 # 4/5 coding rate
implicitHeader = False # implicit header
sf = 7 # 7 spreading factor (default)
checkSum = True # enabling CRC
syncWord = 0x12
power = 17 # max power
preamble = 8 # matching stm

# ALTITUDE (plotted) 2 bytes
# ORIENTATION IN ALL 3 AXES (numbers & plot) 2*3=6 bytes
# LONGITUDE AND LATITUDE (number) 2*8=16 bytes
# VELOCITY IN ALL 3 AXES (plotted) 6 bytes
# STATE (number) 1 byte
# BATTERY VOLTAGE (number) 1 byte
# FRAME COUNTER (number) 1 byte
# TIME SINCE STARTUP (number) 4 bytes 
# 2+6+8+6+1+1+1+4 = 35 bytes total

dummy_frames = [
      [1000,  360.23, 130.1, -100.5,  7500.123, 14000.5182,   0,  0,   0,  0, 100, 0.0],
      [2000,  125.6, 135.6,  140.2,  7501.129681, 14001.123456789,   5, -10, 10,  1, 90, 0.5],
      [4000,  130.0, 10.0,  160.0,  7600.059102305, 14200.01923941,  10, 200, 30,  1, 40, 1.0],
      [8000,  140.0, 100.0,  180.0,  7700.011959123, 14300.631923941,  15, 400, 50,  2, 20, 1.5],
      [12000, 135.0, 140.0,  170.0,  7800.01959123, 14400.0192391,   8, -100, 25,  2, 10, 2.0]
];

count = 0

@dataclass
class Packet:
    altitude: float # will convert to uint16 2 bytes
    orientationX: float # will convert into int16 2 bytes
    orientationY: float # will convert into int16 2 bytes
    orientationZ: float # will convert into int16 2 bytes
    longitude: float # double 8 bytes
    latitude: float # double 8 bytes
    velocityX: float # will convert to int16 2 bytes
    velocityY: float # will convert to int16 2 bytes
    velocityZ: float # will convert to int16 2 bytes
    state: int # uint8 1 byte
    battery_voltage: int # uint8 1 byte
    time_since_startup: float # float 2 bytes
    frame_counter: int = 0 # uint8 1 byte

    # ALTITUDE (plotted) 2 bytes
    # ORIENTATION IN ALL 3 AXES (numbers & plot) 2*3=6 bytes
    # LONGITUDE AND LATITUDE (number) 2*8=16 bytes
    # VELOCITY IN ALL 3 AXES (plotted) 6 bytes
    # STATE (number) 1 byte
    # BATTERY VOLTAGE (number) 1 byte
    # FRAME COUNTER (number) 1 byte
    # TIME SINCE STARTUP (number) 4 bytes 
    # 2+6+16+6+1+1+1+4 = 37 bytes total
        
    # Need to make sure orientation is just degrees from -360 to 360,
    # velocity is from -32768 to 32767, 
    # altitude is from 0 to 65535, 
    # longitude/latitude are reasonable floats that fit in 4 bytes (so like 4 decimals ish?),
    # state, battery voltage, and frame are from 0 to 255 (more than 255 frames would mean it's transmitting for 510 seconds soo probs ok)
    # time since startup is float since it's transmitting at 2 Hz so might need this precision?

    def update(self, frame):
        (
            self.altitude,
            self.orientationX, self.orientationY, self.orientationZ,
            self.longitude, self.latitude,
            self.velocityX, self.velocityY, self.velocityZ,
            self.state, self.battery_voltage,
            self.time_since_startup
        ) = frame
    
    def pack(self):
        return struct.pack('<Hhhh' + 'dd' + 'hhh' + 'BBBf', int(self.altitude), int(self.orientationX), int(self.orientationY), int(self.orientationZ),
                           self.longitude, self.latitude, 
                           int(self.velocityX), int(self.velocityY), int(self.velocityZ),
                           self.state, self.battery_voltage, self.frame_counter, self.time_since_startup)
    
    def send(self):
        packed_data = self.pack()
        loralib.transmit(packed_data)
        self.frame_counter = (self.frame_counter + 1) % 256

if __name__ == "__main__":

    loralib.initialize()
    # configure with: 915 MHz frequency band, 125 kHz bandwidth, 
    # 4/5 coding rate (4/4+cr), no explicit header, spreading factor of 7, enabling CRC (adding a checksum),
    # sync word as 0x12, and outputting at max power
    loralib.configure(fq, bw, cr, implicitHeader, sf, checkSum, syncWord, power, preamble)
    loralib.setContMode(False)
    packet = Packet(*dummy_frames[0])
    print("waiting for start!")
    # This while loop is just for detecting if it gets the start command from the STM
    while startup == False:
                command = loralib.receive()
                if command == "START":
                    startup = True
                    time.sleep(1)
                    loralib.transmit(b"ACK")
                    time.sleep(1)

    # Sends data                     
    while 1:  
        for frame in dummy_frames:
            # idk if .update is the best way to do it 
            packet.update(frame)
            packet.send()
            time.sleep(1)