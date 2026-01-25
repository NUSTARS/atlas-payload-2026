import numpy as np

def simulate_roll_truth(
        t, 
        x0=(0,0,0),
        I=0.05, # moment of inertia
        # Disturbance and control
        tau_dist_std=0.02, # disturbance torque std
        tau_dist_tau=0.15, # correlation time
        tau_max=np.inf,
        Kp=0.0,              # control gain on angle  [N*m/rad]  
        Kd=0.0,
        sigma_gyro=0.05,
        # Bias drift model (random walk)
        bias_rw_std=0.05,  # deg/s
        rng=np.random.default_rng(3)
    ):

    dt = np.diff(t)[0]
    N = len(t)

    phi0, omega0, b0 = x0
    phi = np.zeros(N)
    omega = np.zeros(N)
    b = np.zeros(N)
    phi[0] = phi0
    omega[0] = omega0
    b[0] = b0
    z_gyro = np.zeros(N) # gyro measurements 

    # correlated disturbance torque
    tau_dist = np.zeros(N)
    if tau_dist_tau <= 0:
        tau_dist_tau = 1e-6
    a = np.exp(-dt / tau_dist_tau)
    # Choose driving noise so stationary std is tau_dist_std
    q = tau_dist_std * np.sqrt(1 - a**2)
  
    def clip(x, lim):
        if not np.isfinite(lim):
            return x
        return np.clip(x, -lim, lim)
    
    for k in range(N - 1):
        tau_dist[k+1] = a * tau_dist[k] + rng.normal(0, q) # disturbance torque
        tau_ctrl = clip(-Kp * phi[k] - Kd * omega[k], tau_max) # PD control
        alpha = (tau_ctrl + tau_dist[k]) / I 
        omega[k+1] = omega[k] + alpha * dt # update omega
        phi[k+1] = phi[k] + omega[k+1] * dt  # update phi
        b[k+1] = b[k] + rng.normal(0, bias_rw_std * dt) # bias random walk Qb
        z_gyro[k] = omega[k] + b[k] + rng.normal(0, sigma_gyro) # get gyro measurement of omega 
    
    z_gyro[-1] = omega[-1] + b[-1] + rng.normal(0, sigma_gyro) # last measurement

    return (phi, omega, b, tau_dist, z_gyro)

def simulate_measurements(
    t,
    phi_true,
    omega_true,
    bias_true,
    sigma_gyro,
    rng
):
    dt = np.median(np.diff(t))  # assumes ~uniform sampling
    N = len(t)
    z_rate = omega_true + bias_true + sigma_gyro * rng.standard_normal(N)

    z_angle = np.empty(N)
    z_angle[0] = phi_true[0]
    last_rate = z_rate[0]
    for k in range(N - 1):
        last_rate = z_rate[k]
        z_angle[k+1] = z_angle[k] + last_rate * dt

    return (z_rate, z_angle)




