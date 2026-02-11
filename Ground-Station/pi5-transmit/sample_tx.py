import loralibPi5 as loralib
import time

fq = 915000000 # 915 MHz
bw = 125 # 125 kHz
cr = 1 # 4/5 coding rate
explicitHeader = False # implicit header
sf = 7 # 7 spreading factor (default)
checkSum = False # FOR TESTING NOW NO CHECKSUM, EVENTUALLY MAYBE IT WILL HAVE ONE
syncWord = 0x12
power = 17 # max power

#############################################
if __name__ == "__main__":

    cnt = 0
    loralib.initialize()
    # configure with: 915 MHz frequency band, 125 kHz bandwidth, 
    # 4/5 coding rate (4/4+cr), no explicit header, spreading factor of 7, disabling CRC (adding a checksum),
    # sync word as 0x12, and outputting at max power
    loralib.configure(fq, bw, cr, explicitHeader, sf, checkSum, syncWord, power)
    for i in range(1000):
        print("Sending msg {}".format(i))
        
        loralib.transmit("Hello world {}".format(cnt))
        cnt += 1
        time.sleep(2.5)    

