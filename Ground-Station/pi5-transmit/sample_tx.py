import loralibPi5 as loralib
import time
import struct

fq = 915000000 # 915 MHz
bw = 125 # 125 kHz
cr = 1 # 4/5 coding rate
implicitHeader = True # implicit header
sf = 7 # 7 spreading factor (default)
checkSum = False # FOR TESTING NOW NO CHECKSUM, EVENTUALLY MAYBE IT WILL HAVE ONE
syncWord = 0x12
power = 17 # max power

dummy_frames = [
      [1000,  360, 130, -100,  7500, 14000,   0,  0,   0,  0, 100, 0],
      [2000,  125, 135,  140,  7501, 14001,   5, -10, 10,  1, 90, 1],
      [4000,  130, 10,  160,  7600, 14200,  10, 200, 30,  1, 40, 2],
      [8000,  140, 100,  180,  7700, 14300,  15, 400, 50,  2, 20, 3],
      [12000, 135, 140,  170,  7800, 14400,   8, -100, 25,  2, 10, 4]
];

count = 0

def pack_frame(frame):
    return struct.pack('<' + 'h'*len(frame), *frame)

#############################################
# STM32 CODE TO UNPACK, WE ALREADY ARE UNPACKING IN DATA VIS THO
# int16_t values[12];

# for (int i = 0; i < 12; i++) {
#     values[i] = (int16_t)(data[2*i] | (data[2*i+1] << 8));
# }
#############################################

#############################################
if __name__ == "__main__":

    loralib.initialize()
    # configure with: 915 MHz frequency band, 125 kHz bandwidth, 
    # 4/5 coding rate (4/4+cr), no explicit header, spreading factor of 7, disabling CRC (adding a checksum),
    # sync word as 0x12, and outputting at max power
    loralib.configure(fq, bw, cr, implicitHeader, sf, checkSum, syncWord, power)
    while 1:
        for i, frame in enumerate(dummy_frames):
            print(f"Sending dummy frame {i}")
            print(f"This is msg number {count}")
            packed = pack_frame(frame)
            loralib.transmit(packed)
            count += 1
            time.sleep(0.5)