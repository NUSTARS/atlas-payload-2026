import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter, stft, istft
from scipy.fft import rfft, irfft, rfftfreq

# ============================================================================
# CONFIGURATION
# ============================================================================
INPUT_CSV = 'Plotting/Data/FT1_primary.csv'
OUTPUT_DIR = 'Plotting/Data/Synthetic/'  # Directory to save synthetic data
NUM_SYNTHETIC = 1  # Number of synthetic profiles to generate
FLIGHT_STATE = 7  # Which flight state to analyze
DT = 0.01  # Time step (seconds)
SGF_WINDOW = 20  # Savitzky-Golay filter window length
SGF_POLYORDER = 3  # Savitzky-Golay polynomial order

# STFT Parameters for nonstationary bootstrap
STFT_WINDOW_SIZE = 256  # Window size for STFT (samples)
STFT_OVERLAP = 0.75  # overlap between windows

# ============================================================================
# LOAD AND PREPROCESS DATA
# ============================================================================
print("Loading and preprocessing data...")
df = pd.read_csv(INPUT_CSV)
df = df.set_index('time')
df = df[~df.index.duplicated(keep='first')]

# Resample to uniform time intervals
t_new = np.arange(df.index.min(), df.index.max(), DT)
df = df.reindex(df.index.union(t_new)).interpolate('index').loc[t_new]
df = df.reset_index()

# Filter by flight state
df_main = df[df['state'] == FLIGHT_STATE]
time = df_main['time'].values
roll = df_main['gyro_roll'].values

print(f"Extracted {len(roll)} samples from state {FLIGHT_STATE}")
print(f"Time range: {time[0]:.2f} to {time[-1]:.2f} seconds")

# ============================================================================
# COMPUTE ROLL ACCELERATION FROM REAL DATA
# ============================================================================
print("\nComputing roll acceleration...")
roll_smooth = savgol_filter(roll, window_length=SGF_WINDOW, polyorder=SGF_POLYORDER)
roll_dot = savgol_filter(roll, window_length=SGF_WINDOW, polyorder=SGF_POLYORDER, 
                         deriv=1, delta=DT)

print(f"Max roll acceleration: {max(np.abs(roll_dot)):.2f} °/s²")

# ============================================================================
# COMPUTE STFT (Short-Time Fourier Transform)
# ============================================================================
print("\nComputing STFT for nonstationary analysis...")
nperseg = STFT_WINDOW_SIZE
noverlap = int(nperseg * STFT_OVERLAP)

# Compute STFT of real data
f_stft, t_stft, Zxx = stft(roll_dot, fs=1/DT, nperseg=nperseg, noverlap=noverlap)

print(f"STFT shape: {Zxx.shape}")
print(f"Frequency bins: {len(f_stft)}, Time windows: {len(t_stft)}")
print(f"Time resolution: {t_stft[1] - t_stft[0]:.3f} s")
print(f"Frequency resolution: {f_stft[1] - f_stft[0]:.3f} Hz")

# ============================================================================
# NONSTATIONARY BOOTSTRAP: GENERATE SYNTHETIC SIGNALS
# ============================================================================
print(f"\nGenerating {NUM_SYNTHETIC} synthetic profiles using nonstationary bootstrap...")

import os
os.makedirs(OUTPUT_DIR, exist_ok=True)

synthetic_profiles = []

for i in range(NUM_SYNTHETIC):
    print(f"  Generating synthetic profile {i+1}/{NUM_SYNTHETIC}...")
    
    # Create synthetic STFT with randomized phases but same magnitude
    Zxx_synthetic = np.zeros_like(Zxx, dtype=complex)
    
    # For each time window
    for t_idx in range(Zxx.shape[1]):
        # Get magnitude spectrum for this time window
        magnitude = np.abs(Zxx[:, t_idx])
        
        # Generate random phases (uniform between 0 and 2π)
        random_phases = np.random.uniform(0, 2*np.pi, len(magnitude))
        
        # Preserve DC component phase if it exists
        if magnitude[0] > 0:
            random_phases[0] = np.angle(Zxx[0, t_idx])
        
        # Construct complex spectrum with random phases
        Zxx_synthetic[:, t_idx] = magnitude * np.exp(1j * random_phases)
    
    # Inverse STFT to reconstruct time-domain signal
    _, synthetic_roll_accel = istft(Zxx_synthetic, fs=1/DT, nperseg=nperseg, noverlap=noverlap)
    
    # Trim or pad to match original length
    if len(synthetic_roll_accel) > len(roll_dot):
        synthetic_roll_accel = synthetic_roll_accel[:len(roll_dot)]
    elif len(synthetic_roll_accel) < len(roll_dot):
        synthetic_roll_accel = np.pad(synthetic_roll_accel, 
                                      (0, len(roll_dot) - len(synthetic_roll_accel)), 
                                      mode='edge')
    
    # Store for plotting
    synthetic_profiles.append(synthetic_roll_accel)
    
    # Save to CSV
    output_df = pd.DataFrame({
        'time': time,
        'roll_acceleration': synthetic_roll_accel
    })
    
    output_filename = f'{OUTPUT_DIR}synthetic_roll_accel_{i+1}.csv'
    output_df.to_csv(output_filename, index=False)

print("\nGeneration complete!")

# ============================================================================
# VISUALIZATION
# ============================================================================
print("\nGenerating plots...")

fig = plt.figure(figsize=(12, 12))

# Plot 1: Original roll velocity (raw and smooth)
ax1 = plt.subplot(5, 1, 1)
ax1.plot(time, roll, label='Raw Roll Velocity', linestyle=':', alpha=0.7)
ax1.plot(time, roll_smooth, label='Smooth Roll Velocity', linewidth=2)
ax1.set_ylabel('Roll Rate [°/s]')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_title('Original Flight Data')

# Plot 2: Real roll acceleration
ax2 = plt.subplot(5, 1, 2)
ax2.plot(time, roll_dot, label='Real Roll Acceleration', color='orange', linewidth=2)
ax2.set_ylabel('Roll Accel [°/s²]')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_title('Real Roll Acceleration (from Flight Data)')

# Plot 3: Spectrogram of real data
ax3 = plt.subplot(5, 1, 3)
magnitude_spectrogram = np.abs(Zxx)
im = ax3.pcolormesh(t_stft, f_stft, magnitude_spectrogram, 
                     shading='gouraud', cmap='viridis')
ax3.set_ylabel('Frequency [Hz]')
ax3.set_ylim(0, 20)  # Focus on relevant frequencies
ax3.set_title('Spectrogram of Real Roll Acceleration')
plt.colorbar(im, ax=ax3, label='Magnitude')

# Plot 4: Synthetic roll accelerations
ax4 = plt.subplot(5, 1, 4)
for i, synth in enumerate(synthetic_profiles):
    ax4.plot(time, synth, alpha=0.6, label=f'Synthetic {i+1}')
ax4.set_ylabel('Roll Accel [°/s²]')
ax4.set_xlabel('Time [s]')
ax4.legend()
ax4.grid(True, alpha=0.3)
ax4.set_title(f'Synthetic Roll Accelerations (n={NUM_SYNTHETIC})')

# Plot 5: Spectrogram of one synthetic example
ax5 = plt.subplot(5, 1, 5)
_, _, Zxx_synth_plot = stft(synthetic_profiles[0], fs=1/DT, nperseg=nperseg, noverlap=noverlap)
magnitude_spectrogram_synth = np.abs(Zxx_synth_plot)
im2 = ax5.pcolormesh(t_stft, f_stft, magnitude_spectrogram_synth, 
                      shading='gouraud', cmap='viridis')
ax5.set_ylabel('Frequency [Hz]')
ax5.set_xlabel('Time [s]')
ax5.set_ylim(0, 20)
ax5.set_title('Spectrogram of Synthetic Roll Acceleration (Example 1)')
plt.colorbar(im2, ax=ax5, label='Magnitude')

plt.tight_layout()
plt.savefig(f'{OUTPUT_DIR}synthetic_analysis_spectrogram.png', dpi=150)
print(f"Plot saved: {OUTPUT_DIR}synthetic_analysis_spectrogram.png")
plt.show()

# ============================================================================
# STATISTICS COMPARISON
# ============================================================================
print("\n" + "="*60)
print("STATISTICS COMPARISON")
print("="*60)
print(f"{'Metric':<30} {'Real':<15} {'Synthetic (avg)':<15}")
print("-"*60)
print(f"{'Mean [°/s²]':<30} {np.mean(roll_dot):>14.3f} {np.mean([np.mean(s) for s in synthetic_profiles]):>14.3f}")
print(f"{'Std Dev [°/s²]':<30} {np.std(roll_dot):>14.3f} {np.mean([np.std(s) for s in synthetic_profiles]):>14.3f}")
print(f"{'Max Absolute [°/s²]':<30} {np.max(np.abs(roll_dot)):>14.3f} {np.mean([np.max(np.abs(s)) for s in synthetic_profiles]):>14.3f}")
print(f"{'RMS [°/s²]':<30} {np.sqrt(np.mean(roll_dot**2)):>14.3f} {np.mean([np.sqrt(np.mean(s**2)) for s in synthetic_profiles]):>14.3f}")
print("="*60)

# ============================================================================
# TIME-VARYING SPECTRAL COMPARISON
# ============================================================================
print("\n" + "="*60)
print("TIME-VARYING SPECTRAL CONTENT PRESERVED")
print("="*60)
print("The spectrograms show that synthetic signals preserve the")
print("time-varying frequency characteristics of the real data.")
print("Each time window has the same magnitude spectrum but with")
print("randomized phases, creating realistic variations.")
print("="*60)