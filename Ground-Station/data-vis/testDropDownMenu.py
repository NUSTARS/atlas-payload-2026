import serial
import matplotlib.pyplot as plt
import time
import csv
from datetime import datetime
import os
import struct
from matplotlib.widgets import Button

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
SAVE_DATA = False       # CHANGE BACK TO TRUE WHEN THE DATA IS REAL

numBytes = 12
frameSize = numBytes*2

altitudePlot = 0
orientationPlot = [1, 2, 3] # and numbers!
longlatitudes = [4, 5]
velocityPlot = [6, 7, 8]
state = [9]
batteryVoltage = [10]
frameCounter = [11]

statesInText = ["Standby", 'Launching', "Apogee(?)"]

bigNumberDisplayOnly = orientationPlot + longlatitudes + state + batteryVoltage + frameCounter

# --------------------------
# Functions
# --------------------------
def parse_message(msg):
    if len(msg) != 24:
        return None
    return struct.unpack('<12h', msg)  # little-endian, 12 signed int16

# --------------------------
# Setup Serial
# --------------------------
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)

# --------------------------
# Setup Matplotlib for numbers
# --------------------------

fig = plt.figure(figsize=(14, 8))

def close_callback(event):
    plt.close(fig)  # this will exit your main loop

gs = fig.add_gridspec(5, 2, width_ratios=[3, 1])

# Left side plots
ax_alt = fig.add_subplot(gs[0, 0])
ax_orient = fig.add_subplot(gs[2, 0])
ax_vel = fig.add_subplot(gs[4, 0])

# Right side numbers panel
ax_nums = fig.add_subplot(gs[:, 1])
ax_nums.axis('off')

# Altitude
alt_data = []
alt_line, = ax_alt.plot([])
ax_alt.set_title("Altitude")
ax_alt.set_ylim(0, 15000)

# Orientation
orient_data = [[0]*50 for _ in range(3)]
colors_orient = ['red', 'green', 'blue']  # X=red, Y=green, Z=blue
orient_lines = [ax_orient.plot([], [], color=c)[0] for c in colors_orient]
ax_orient.set_title("Orientation (X,Y,Z)")
ax_orient.legend(['X', 'Y', 'Z'], loc='upper left')

# Velocity
vel_data = [[0]*50 for _ in range(3)]
colors_vel = ['red', 'green', 'blue']  # X=red, Y=green, Z=blue
vel_lines = [ax_vel.plot([], [], color=c)[0] for c in colors_vel]
ax_vel.set_title("Velocity (X,Y,Z)")
ax_vel.legend(['Vx', 'Vy', 'Vz'], loc='upper left')

number_titles = [
    'Orientation X', 'Orientation Y', 'Orientation Z',
    'Longitude', 'Latitude',
    'State', 'Battery Voltage', 'Frame Counter'
]

y_positions = [0.9, 0.8, 0.7, 0.55, 0.45, 0.3, 0.2, 0.1]

colors = ['red', 'green', 'blue', 
          'black', 'black', 
          'black', 'green', 'black']

number_texts = []
title_texts = []

for y, title, c in zip(y_positions, number_titles, colors):
    title_obj = ax_nums.text(0.5, y+0.04, title,
                             ha='center', fontsize=10, color=c)
    value_obj = ax_nums.text(0.5, y,
                             '', ha='center', fontsize=18, color=c)

    title_texts.append(title_obj)
    number_texts.append(value_obj)

ax_button = plt.axes([0.8, 0.01, 0.1, 0.05])
btn = Button(ax_button, 'Exit')
btn.on_clicked(close_callback)

fig.canvas.draw()
fig.canvas.flush_events()

plt.ion()
plt.show()

# --------------------------
# Setup CSV Logging
# --------------------------
# Get directory where this script lives
if SAVE_DATA == True:
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Create a subfolder inside it
    log_folder = os.path.join(base_dir, "csvFolders")

    # Make folder if it doesn't exist
    os.makedirs(log_folder, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"telemetry_log_{timestamp}.csv"

    full_path = os.path.join(log_folder, csv_filename)

    csv_file = open(full_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)

    # Write header row
    csv_writer.writerow([
        "Time",
        "Altitude",
        "OrientX", "OrientY", "OrientZ",
        "Longitude", "Latitude",
        "VelX", "VelY", "VelZ",
        "State",
        "Battery",
        "Frame"
    ])

# --------------------------
# Main loop
# --------------------------

try:
    while plt.fignum_exists(fig.number):
        rowmsg = ser.read(frameSize)
        if len(rowmsg) != frameSize:
            continue

        ch_values = parse_message(rowmsg)
        if len(ch_values) != numBytes:
            continue

        battery_value = ch_values[batteryVoltage[0]]

        if battery_value < 20:
            title_texts[6].set_color('red')  # battery index
            number_texts[6].set_color('red')  # battery index
        else:
            title_texts[6].set_color('green')  # battery index
            number_texts[6].set_color('green')

        if SAVE_DATA == True:
            current_time = datetime.now().strftime("%H:%M:%S.%f")

            csv_writer.writerow([
                current_time,
                ch_values[0],
                ch_values[1], ch_values[2], ch_values[3],
                ch_values[4], ch_values[5],
                ch_values[6], ch_values[7], ch_values[8],
                ch_values[9],
                ch_values[10],
                ch_values[11]
            ])

            csv_file.flush()

        # -----------------------
        # Update Big Numbers
        # -----------------------

        for txt, idx in zip(number_texts, bigNumberDisplayOnly):
            if idx == state[0]:  # if this is the state value
                state_index = ch_values[idx]
                # Make sure index is valid
                if 0 <= state_index < len(statesInText):
                    txt.set_text(statesInText[state_index])
                else:
                    txt.set_text(f"Unknown ({state_index})")
            else:
                txt.set_text(str(ch_values[idx]))

        # for txt, idx in zip(number_texts, bigNumberDisplayOnly):
        #     txt.set_text(str(ch_values[idx]))

        # -----------------------
        # Update Altitude Plot
        # -----------------------
        alt_data.append(ch_values[altitudePlot])
        if len(alt_data) > 50:
            alt_data.pop(0)

        alt_line.set_data(range(len(alt_data)), alt_data)
        ax_alt.set_xlim(0, 50)

        # Auto-scale Y-axis with 10% padding
        if alt_data:
            y_min = min(alt_data)
            y_max = max(alt_data)
            padding = 0.1 * (y_max - y_min) if (y_max - y_min) != 0 else 1
            ax_alt.set_ylim(y_min - padding, y_max + padding)

        # -----------------------
        # Update Orientation Plot
        # -----------------------
        for i in range(3):
            orient_data[i].append(ch_values[orientationPlot[i]])
            if len(orient_data[i]) > 50:
                orient_data[i].pop(0)

            orient_lines[i].set_data(range(len(orient_data[i])), orient_data[i])

        ax_orient.set_xlim(0, 50)

        # Auto-scale Y-axis with 10% padding
        all_orient = [val for sublist in orient_data for val in sublist]
        if all_orient:
            y_min = min(all_orient)
            y_max = max(all_orient)
            padding = 0.1 * (y_max - y_min) if (y_max - y_min) != 0 else 1
            ax_orient.set_ylim(y_min - padding, y_max + padding)

        # -----------------------
        # Update Velocity Plot
        # -----------------------
        for i in range(3):
            vel_data[i].append(ch_values[velocityPlot[i]])
            if len(vel_data[i]) > 50:
                vel_data[i].pop(0)

            vel_lines[i].set_data(range(len(vel_data[i])), vel_data[i])

        ax_vel.set_xlim(0, 50)

        # Auto-scale Y-axis with 10% padding
        all_vel = [val for sublist in vel_data for val in sublist]
        if all_vel:
            y_min = min(all_vel)
            y_max = max(all_vel)
            padding = 0.1 * (y_max - y_min) if (y_max - y_min) != 0 else 1
            ax_vel.set_ylim(y_min - padding, y_max + padding)

        # -----------------------
        # Refresh ONE figure
        # -----------------------
        plt.pause(0.001)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    if SAVE_DATA == True:
        csv_file.close()
    plt.ioff()
    plt.show()
