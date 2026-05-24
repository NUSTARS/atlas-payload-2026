import serial
import matplotlib.pyplot as plt
import time
import csv
from datetime import datetime
import os
import struct
from matplotlib.widgets import Button
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk


# ALTITUDE (plotted) 4 bytes
# ORIENTATION IN ALL 3 AXES (numbers & plot) 3*4=12 bytes
# LONGITUDE AND LATITUDE (neaumber) 2*8=16 bytes
# VELOCITY IN ALL 3 AXES (plotted) 12 bytes
# STATE (number) 1 byte
# BATTERY VOLTAGE (number) 4 byte
# FRAME COUNTER (number) 2 bytes
# TIME SINCE STARTUP (number) 4 bytes 
# above is unupdated
# 4+12+16+12+1+4+2+4 = 56 bytes total

# TAKE DIFFERENCE BETWEEN STARTING LAT AND LONG BETWEEN CURRENT
# VERTICAL VELOCITY

# ADD BIG NUMBER FOR VELOCITY AND ALTITUDE

# heading and tilt instead of orientation

# --------------------------
# Configuration
# --------------------------
ports = [port.device for port in serial.tools.list_ports.comports()]

selected_port = None

def start():
    global selected_port
    selected_port = combo.get()
    root.destroy()

# Create selection window
root = tk.Tk()
root.title("Select COM Port")

tk.Label(root, text="Choose COM Port:").pack(pady=5)

combo = ttk.Combobox(root, values=ports)
combo.pack(pady=5)

tk.Button(root, text="Start", command=start).pack(pady=10)

root.mainloop()

print("Selected:", selected_port)

COM_PORT = selected_port      
BAUD_RATE = 115200      # match STM32 UART baud rate
TIMEOUT = 1             # seconds
SHOW_GRAPHS = True      # set to True to show scrolling line plots
SAVE_DATA = True       # CHANGE BACK TO TRUE WHEN THE DATA IS REAL

firstDataPoint = True

numVars = 15
frameSize = 58

altitudePlot = 0
orientationPlot = [1, 2, 3] # and numbers!
velocityPlot = [4, 5, 6]
batteryVoltage = [7]
timeSinceStartup = [8]
longlatitudes = [9, 10]
state = [11]
frameCounter = [12]


startup = False

# change these to whatever states we actually choose
statesInText = ["Launchpad", "Boost", "Apogee", "Running Controls", "Landed"]

bigNumberDisplayOnly = orientationPlot + longlatitudes + state + batteryVoltage + frameCounter + timeSinceStartup + [0] + [6] + [15] + [16] + [17] + [18]

# --------------------------
# Functions
# --------------------------
def parse_message(msg):
    if len(msg) != frameSize:
        return None
    return struct.unpack('<fffffffff' + 'dd' + 'HH' + 'BB', msg)  # same packing as in sample tx

# --------------------------
# Setup Serial
# --------------------------

# To startup payload to make it start sending data, you press enter
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
input("Press ENTER to send START command to STM: ")
ser.write(b"START")
print("Command sent. Waiting for LoRa handshake...")

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
    'Yaw', 'Pitch', 'Roll',
    'Longitude', 'Latitude',
    'State', 'Battery Voltage', 'Frame Counter', "Time Since Startup",
    'Altitude', 'Velocity (Y)', 'Longitude Change', 'Latitude Change',
    'RSSI (dBm)', 'SNR (dB)'
]

x_positions = [.05,.05,.05,
               .5,.5,
               1,1,.05,.05,
               1,1,.5,.5,
               1, 1]

y_positions = [0.92, 0.82, 0.72, 
               0.60, 0.52, 
               .10,0, 0.10, 0,
               .92,.82,.40,.30,
               .40, .30]

colors = ['red', 'green', 'blue', 
          'black', 'black', 
          'black', 'green', 'black', 'black',
          'red', 'red', 'red', 'red',
          'gray', 'gray']

number_texts = []
title_texts = []

for x, y, title, c in zip(x_positions, y_positions, number_titles, colors):
    title_obj = ax_nums.text(x, y+0.04, title,
                             ha='center', fontsize=10, color=c)
    value_obj = ax_nums.text(x, y,
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

    timestamp = datetime.now().strftime("%b%d_%H%M")
    csv_filename = f"flightdata_{timestamp}.csv"

    full_path = os.path.join(log_folder, csv_filename)

    csv_file = open(full_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)

    # Write header row
    csv_writer.writerow([
        "Time",
        "Altitude",
        "OrientX", "OrientY", "OrientZ",
        "VelX", "VelY", "VelZ",
        "Battery",
        "TimeSinceStartup",
        "Longitude", "Latitude",
        "State",
        "Frame",
        'Longitude Change', 'Latitude Change',
        "RSSI", "SNR"
    ])

# --------------------------
# Main loop
# --------------------------
try:
    while startup == False:
        if ser.in_waiting > 0:
            # Use readline() because STM32 printf() ends with \r\n
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line:
                print(f"[STM32]: {line}")
                
            # Checks that STM received the correct ACK
            if "Starting normal routine" in line:
                print("Handshake confirmed! Opening plots...")
                startup = True
                time.sleep(1) # Small buffer to let UART clear
                ser.reset_input_buffer() # Clear the "ACK" text so it doesn't mess up binary parsing

            if "Retry please" in line:
                input("Press ENTER to send START command to STM: ")
                ser.write(b"START")
                print("Command sent. Waiting for LoRa handshake...")
            
    while plt.fignum_exists(fig.number):
        rowmsg = ser.read(frameSize)
        if len(rowmsg) != frameSize:
            continue

        ch_values = parse_message(rowmsg)
        if len(ch_values) != numVars:
            continue

        raw_rssi = ch_values[13]
        raw_snr = ch_values[14]

        # 915MHz math: RSSI = PacketRssi - 157
        actual_rssi = raw_rssi - 157
        # SNR = PacketSnr / 4
        actual_snr = raw_snr / 4
    
        if firstDataPoint == True:
            startingLong = ch_values[longlatitudes[0]]
            startingLat = ch_values[longlatitudes[1]]
            firstDataPoint = False

        ch_values = ch_values + ((ch_values[longlatitudes[0]] - startingLong),)
        ch_values = ch_values + ((ch_values[longlatitudes[1]] - startingLat),)

        ch_values = ch_values + (actual_rssi,) # Index 17
        ch_values = ch_values + (actual_snr,)  # Index 18

        # if abs(ch_values[orientationPlot[0]]) > 1000:
        #     continue

        print(ch_values)

        battery_value = ch_values[batteryVoltage[0]]

        if battery_value < 14:
            title_texts[batteryVoltage[0]-1].set_color('red')  # battery index
            number_texts[batteryVoltage[0]-1].set_color('red')  # battery index
        else:
            title_texts[batteryVoltage[0]-1].set_color('green')  # battery index
            number_texts[batteryVoltage[0]-1].set_color('green')

        if SAVE_DATA == True:
            current_time = datetime.now().strftime("%H:%M:%S.%f")

            csv_writer.writerow([
                current_time,
                ch_values[0],
                ch_values[1], ch_values[2], ch_values[3],
                ch_values[4], ch_values[5], ch_values[6], 
                ch_values[7], 
                ch_values[8],
                ch_values[9], ch_values[10],
                ch_values[11],
                ch_values[12],
                ch_values[15], ch_values[16],
                actual_rssi, actual_snr
            ])

            csv_file.flush()

        ch_values = [round(x,2) for x in ch_values] 

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
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    if SAVE_DATA == True:
        csv_file.close()
    plt.ioff()
    plt.show()
