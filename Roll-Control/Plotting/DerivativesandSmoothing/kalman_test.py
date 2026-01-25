import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

csv_path = 'atlas-payload-2026/Roll-Control/Plotting/DerivativesandSmoothing/maryland_data.csv'#"Roll-Control\\Plotting\\DerivativesandSmoothing\\maryland_data.csv"
dt = 0.001  # desired time step in seconds


class KalmanSys:
    def __init__(self, A, H, Q, R, x0, P0):
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.K = 0
        self.x = x0
        self.P = P0
        self.innovation = 0

    def update(self):
        #print(np.linalg.norm(self.innovation))
        if(np.linalg.norm(self.innovation) > 15):
            R_curr = self.R * 1.0
            Q_curr = self.Q * 1.0
        else:
            R_curr = self.R * 1.0
            Q_curr = self.Q * 1.0


        self.x = np.dot(self.A, self.x) # predict next step
        self.P = np.dot(self.A, np.dot(self.P, np.transpose(self.A))) + Q_curr #predict uncertainty in next step
        self.K = np.dot(np.dot(self.P,np.transpose(self.H)), np.linalg.inv(np.dot(self.H,np.dot(self.P, np.transpose(H))) + R_curr))
        self.P = self.P - np.dot(self.K, np.dot(self.H, self.P))

    def estimate(self, z): # z is the measurement
        self.innovation = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(self.K, self.innovation)
        return self.x

def input_data(csv_path, dt):

    df = pd.read_csv(csv_path)

    #df = df.dropna()
    df["time"] = df["Time (ms)"]/10  # convert to seconds
    df["roll"] = df["IMU AngVeloX"]
    df['heading'] = df["IMU Roll"]

    df = df[["time","roll","heading"]]
    df = df.sort_values("time")
    df = df[df['time'] > 1.0]
    df = df[df['time'] < 2.0]
    df["time"] = pd.to_numeric(df["time"])
    df = df.set_index("time")


    # Create uniform time grid
    t_uniform = np.arange(df.index.min(),df.index.max(),dt)
    combined_index = sorted(set(list(df.index) + list(t_uniform)))
    df_interp = df.reindex(combined_index).interpolate(method='linear')
    df_uniform = df_interp.loc[t_uniform]

    return df_uniform


df = input_data(csv_path, dt)

time_vals = df.index.to_numpy()
roll_vals = df['roll'].to_numpy()
heading_vals = df['heading'].to_numpy()

filtered_roll_accel = savgol_filter(roll_vals, window_length=20, polyorder=3, deriv=1, delta=dt)

# plt.plot(df.index, df["roll"], label="Interpolated Roll")
# plt.plot(df.index, df["heading"], label="Interpolated Heading")
# plt.show()

# state transition matrix
A = 1 * np.array([[1, dt, 0.5*dt**2],
                [0, 1, dt],
                [0, 0, 0.98]])
# state measurement matrix
H = np.array([[1, 0, 0],
              [0, 1, 0]],)
# state transition noise
Q = 700000 * np.array([[(dt**3)/20, (dt**4)/8, (dt**3)/6],
                    [(dt**4)/8, (dt**3)/3, (dt**2)/2],
                    [(dt**3)/6, (dt**2)/2, dt]])
# state measurement noise
R = 0.00003 * np.array([[1, 0],
              [0, 100]])
# initial state
x0 = np.array([heading_vals[0], roll_vals[0], 0])

# initial covariance matrix
P0 = np.array([[0.1, 0, 0],
               [0, 10, 0],
               [0, 0, 100]])

KS = KalmanSys(A, H, Q, R, x0, P0)

predictions = []

for time, roll, heading in zip(time_vals, roll_vals, heading_vals):
    KS.update()
    prediction = KS.estimate([heading, roll])
    predictions.append(prediction)

predictions = np.array(predictions)

fig, axs = plt.subplots(3,1, figsize=(10, 8))
axs[0].plot(time_vals, predictions[:,0], label="Kalman Filtered Heading")
axs[0].plot(time_vals, heading_vals, label="Original Heading", linestyle='--')
axs[0].legend()

axs[1].plot(time_vals, predictions[:,1], label="Kalman Filtered Roll")
axs[1].plot(time_vals, roll_vals, label="Original Roll", linestyle='--')
axs[1].legend()

axs[2].plot(time_vals, predictions[:,2], label="Kalman Filtered Angular Acceleration")
axs[2].plot(time_vals, filtered_roll_accel, label="SG filtered Angular Acceleration", linestyle='--')
axs[2].legend()
plt.show()

    

