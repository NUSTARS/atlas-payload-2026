import loralibPi5 as loralib
import time
import struct
from dataclasses import dataclass

# LoRa 
fq = 915000000 # 915 MHz
bw = 125 # 125 kHz
cr = 1 # 4/5 coding rate
implicitHeader = True # implicit header
sf = 7 # 7 spreading factor (default)
checkSum = False # FOR TESTING NOW NO CHECKSUM, EVENTUALLY MAYBE IT WILL HAVE ONE
syncWord = 0x12
power = 17 # max power