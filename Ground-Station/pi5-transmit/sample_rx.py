import loralibPi5 as loralib
import time

fq = 915000000 # 915 MHz
bw = 125 # 125 kHz
cr = 1 # 4/5 coding rate
implicitHeader = False # explicit header
sf = 7 # 7 spreading factor (default)
checkSum = True # enabling CRC
syncWord = 0x12
power = 17 # max power
preamble = 8 # matching stm

if __name__ == "__main__":
    print("Starting LoRa RX")
    loralib.initialize()
    # configure with: 915 MHz frequency band, 125 kHz bandwidth, 
    # 4/5 coding rate (4/4+cr), no explicit header, spreading factor of 7, enabling CRC (adding a checksum),
    # sync word as 0x12, and outputting at max power
    loralib.configure(fq, bw, cr, implicitHeader, sf, checkSum, syncWord, power, preamble)
    loralib.setContMode(False)
    while (1):
        meow = loralib.receive()
        time.sleep(.1)
    