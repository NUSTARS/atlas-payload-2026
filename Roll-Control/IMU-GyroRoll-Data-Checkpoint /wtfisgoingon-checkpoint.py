import pandas as pd

import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('Plotting\wtfisgoingon.csv')

# Extract the columns
time = data['time']
gyro_roll = data['gyro_roll']
inter_time = data['inter_time']
inter_gyro_roll = data['inter_gyro_roll']

plt.figure(figsize=(10, 6))
plt.plot(time, gyro_roll, label='Original Gyro Roll')
plt.plot(inter_time, inter_gyro_roll, label='Interpolated Gyro Roll')

plt.xlabel('Time (s)')
plt.ylabel('Roll Rate (deg/s)')
plt.title('Original vs Interpolated Gyro Roll Data')
plt.legend()
plt.grid(True)
plt.show()