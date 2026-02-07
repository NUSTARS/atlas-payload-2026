import serial
import matplotlib.pyplot as plt
import time

# THINGS WE ARE READING: 
# ALTITUDE (plotted) 1 byte
# ORIENTATION IN ALL 3 AXES (numbers & plot) 3 bytes
# LONGITUDE AND LATITUDE (number) 2 bytes
# VELOCITY IN ALL 3 AXES (plotted) 3 bytes
# STATE (number) 1 byte
# BATTERY VOLTAGE (number) 1 byte
# FRAME COUNTER (number) 1 byte

# 1+3+2+3+1+1+1 = 12 bytes total, can change if needed

# --------------------------
# Configuration
# --------------------------
COM_PORT = '/dev/cu.usbmodem103'       # change to your Nucleo COM port
BAUD_RATE = 115200      # match STM32 UART baud rate
TIMEOUT = 1             # seconds
SHOW_GRAPHS = True      # set to True to show scrolling line plots

numBytes = 12

altitudePlot = 0
orientationPlot = [1, 2, 3] # and numbers!
longlatitudes = [4, 5]
velocityPlot = [6, 7, 8]
state = [9]
batteryVoltage = [10]
frameCounter = [11]

bigNumberDisplayOnly = orientationPlot + longlatitudes + state + batteryVoltage + frameCounter

# --------------------------
# Functions
# --------------------------
def parse_message(msg):
    """
    Parses STM32 message like b'#D#0x12#0xF1#0x00#0x03#\n'
    Returns a list of numBytes integers converted from hex
    """
    if len(msg) < numBytes:
        return [0] * numBytes

    return list(msg[:numBytes])

# --------------------------
# Setup Serial
# --------------------------
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)

# --------------------------
# Setup Matplotlib for numbers
# --------------------------

fig_nums, ax_nums = plt.subplots(figsize=(14, 6))
ax_nums.axis('off')

colors = ['cyan', 'cyan', 'cyan', 'black',
          'purple', 'purple', 'green', 'black']

titles = [
    'Orientation X', 'Orientation Y', 'Orientation Z', 'State',
    'Longitude', 'Latitude', 'Battery Voltage', 'Frame Counter'
]

# 2 rows × 4 columns positions
x_positions = [0.125, 0.375, 0.625, 0.875]
y_positions = [0.65, 0.25]

positions = [(x, y) for y in y_positions for x in x_positions]

# Big number text objects
number_texts = [
    ax_nums.text(x, y, '', fontsize=40, ha='center', va='center', color=c)
    for (x, y), c in zip(positions, colors)
]

# Titles slightly above numbers
title_texts = [
    ax_nums.text(x, y + 0.15, t, fontsize=16, ha='center', va='center', color=c)
    for (x, y), t, c in zip(positions, titles, colors)
]

plt.ion()
plt.show()

# --------------------------
# Setup real-time graphs
# --------------------------

# Altitude
fig_alt, ax_alt = plt.subplots()
alt_data = []
alt_line, = ax_alt.plot([], [])
ax_alt.set_title("Altitude")
ax_alt.set_ylim(0, 255)
ax_alt.set_xlim(0, 50)

# Orientation
fig_orient, ax_orient = plt.subplots()
orient_data = [[0]*50 for _ in range(3)]

orient_lines = [
    ax_orient.plot([], [])[0]
    for _ in range(3)
]

ax_orient.set_title("Orientation (X, Y, Z)")
ax_orient.set_ylim(0, 255)
ax_orient.set_xlim(0, 50)

# Velocity
fig_vel, ax_vel = plt.subplots()
vel_data = [[0]*50 for _ in range(3)]

vel_lines = [
    ax_vel.plot([], [])[0]
    for _ in range(3)
]

ax_vel.set_title("Velocity (X, Y, Z)")
ax_vel.set_ylim(0, 255)
ax_vel.set_xlim(0, 50)

# --------------------------
# Main loop
# --------------------------
try:
    while True:
        rowmsg = ser.read(numBytes)
        if not rowmsg: 
            continue 
        ch_values = parse_message(rowmsg)
        if len(ch_values) != numBytes:
            continue

        # Update big numbers
        for txt, idx in zip(number_texts, bigNumberDisplayOnly):
            txt.set_text(str(ch_values[idx]))

        fig_nums.canvas.draw()
        fig_nums.canvas.flush_events()

        # Update altitude graph
        alt_data.append(ch_values[altitudePlot])
        if len(alt_data) > 50:
            alt_data.pop(0)

        alt_line.set_data(range(len(alt_data)), alt_data)
        fig_alt.canvas.draw()
        fig_alt.canvas.flush_events()

        # Update orientation graph
        for i in range(3):
            orient_data[i].append(ch_values[1+i])
            if len(orient_data[i]) > 50:
                orient_data[i].pop(0)

            orient_lines[i].set_data(range(len(orient_data[i])), orient_data[i])

        fig_orient.canvas.draw()
        fig_orient.canvas.flush_events()


        # Update velocity graph
        for i in range(3):
            vel_data[i].append(ch_values[6+i])
            if len(vel_data[i]) > 50:
                vel_data[i].pop(0)

            vel_lines[i].set_data(range(len(vel_data[i])), vel_data[i])

        fig_vel.canvas.draw()
        fig_vel.canvas.flush_events()

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
    plt.show()
