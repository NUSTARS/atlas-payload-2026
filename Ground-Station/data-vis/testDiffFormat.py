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

fig = plt.figure(figsize=(14, 8))

gs = fig.add_gridspec(3, 2, width_ratios=[3, 1])

# Left side plots
ax_alt = fig.add_subplot(gs[0, 0])
ax_orient = fig.add_subplot(gs[1, 0])
ax_vel = fig.add_subplot(gs[2, 0])

# Right side numbers panel
ax_nums = fig.add_subplot(gs[:, 1])
ax_nums.axis('off')

# Altitude
alt_data = []
alt_line, = ax_alt.plot([])
ax_alt.set_title("Altitude")
ax_alt.set_ylim(0, 255)

# Orientation
orient_data = [[0]*50 for _ in range(3)]
orient_lines = [ax_orient.plot([])[0] for _ in range(3)]
ax_orient.set_title("Orientation (X,Y,Z)")
ax_orient.set_ylim(0, 255)

# Velocity
vel_data = [[0]*50 for _ in range(3)]
vel_lines = [ax_vel.plot([])[0] for _ in range(3)]
ax_vel.set_title("Velocity (X,Y,Z)")
ax_vel.set_ylim(0, 255)

number_titles = [
    'Orientation X', 'Orientation Y', 'Orientation Z',
    'Longitude', 'Latitude',
    'State', 'Battery Voltage', 'Frame Counter'
]

y_positions = [0.9, 0.8, 0.7, 0.55, 0.45, 0.3, 0.2, 0.1]

number_texts = []
for y, title in zip(y_positions, number_titles):
    ax_nums.text(0.5, y+0.04, title,
                 ha='center', fontsize=10)
    txt = ax_nums.text(0.5, y,
                       '', ha='center', fontsize=18)
    number_texts.append(txt)

fig.canvas.draw()
fig.canvas.flush_events()

plt.ion()
plt.show()

# --------------------------
# Main loop
# --------------------------
try:
    while True:
        rowmsg = ser.read(numBytes)
        if len(rowmsg) != numBytes:
            continue

        ch_values = parse_message(rowmsg)

        # -----------------------
        # Update Big Numbers
        # -----------------------
        for txt, idx in zip(number_texts, bigNumberDisplayOnly):
            txt.set_text(str(ch_values[idx]))

        # -----------------------
        # Update Altitude Plot
        # -----------------------
        alt_data.append(ch_values[altitudePlot])
        if len(alt_data) > 50:
            alt_data.pop(0)

        alt_line.set_data(range(len(alt_data)), alt_data)
        ax_alt.set_xlim(0, 50)

        # -----------------------
        # Update Orientation Plot
        # -----------------------
        for i in range(3):
            orient_data[i].append(ch_values[orientationPlot[i]])
            if len(orient_data[i]) > 50:
                orient_data[i].pop(0)

            orient_lines[i].set_data(range(len(orient_data[i])), orient_data[i])

        ax_orient.set_xlim(0, 50)

        # -----------------------
        # Update Velocity Plot
        # -----------------------
        for i in range(3):
            vel_data[i].append(ch_values[velocityPlot[i]])
            if len(vel_data[i]) > 50:
                vel_data[i].pop(0)

            vel_lines[i].set_data(range(len(vel_data[i])), vel_data[i])

        ax_vel.set_xlim(0, 50)

        # -----------------------
        # Refresh ONE figure
        # -----------------------
        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    plt.ioff()
    plt.show()
