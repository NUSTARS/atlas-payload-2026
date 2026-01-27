import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import pandas as pd

# Read data from CSV file
df = pd.read_csv('Roll-Control\\VN200 Test\\Roll Test\\VNYMR.csv')

# Convert columns to numeric, handling any non-numeric values
roll = pd.to_numeric(df['Yaw (degrees)'], errors='coerce').values
gyroZ = pd.to_numeric(df['GyroZ (rad/s)'], errors='coerce').values

# Unwrap roll to make it continuous (handle wrapping at 360 degrees)
roll = np.rad2deg(np.unwrap(np.deg2rad(roll)))

# Remove any NaN values
valid_indices = ~(np.isnan(roll) | np.isnan(gyroZ))
roll = roll[valid_indices]
gyroZ = gyroZ[valid_indices]

# Create time vector (assuming constant sampling rate)
# If you have actual timestamps, replace this with your time data
dt = 0.02  # assumed time step in seconds, adjust as needed
t = np.arange(len(roll)) * dt

# Convert gyroZ from rad/s to deg/s for integration
# gyroZ_deg = np.degrees(gyroZ)
gyroZ_deg = gyroZ

# Integrate gyroZ to get roll position
# Using cumulative trapezoidal integration
roll_integrated = integrate.cumulative_trapezoid(gyroZ_deg, t, initial=0)
# Add initial roll value to integrated data
roll_integrated += roll[0]

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Top subplot: Roll and integrated roll
ax1.plot(t, roll, 'b-', label='Roll (measured)', linewidth=1.5)
ax1.plot(t, roll_integrated, 'r--', label='Roll (integrated from gyroZ)', linewidth=1.5, alpha=0.7)
ax1.set_ylabel('Roll Angle (degrees)', fontsize=12)
ax1.set_title('Roll Angle Comparison', fontsize=14, fontweight='bold')
ax1.legend(loc='best')
ax1.grid(True, alpha=0.3)

# Bottom subplot: Roll rate (gyroZ)
ax2.plot(t, gyroZ_deg, 'g-', label='Roll Rate (gyroZ)', linewidth=1.5)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Roll Rate (deg/s)', fontsize=12)
ax2.set_title('Roll Rate', fontsize=14, fontweight='bold')
ax2.legend(loc='best')
ax2.grid(True, alpha=0.3)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()