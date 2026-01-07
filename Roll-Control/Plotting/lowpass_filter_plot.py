"""
Low-pass filter and plot telemetry column (default: acceleration).
Usage: run this script in the repository root or from the Plotting folder.
It will read FT4_Primary.csv by default, apply either a Butterworth filter
(if SciPy is installed) or a rolling mean, and save a comparison PNG in
`Plotting/Plots/lowpass_acceleration.png`.
"""

import os
import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def butter_lowpass_filter(data, fs, cutoff, order=4):
    """Apply zero-phase Butterworth lowpass with filtfilt if available.
    If SciPy is not installed this function will raise ImportError.
    Inputs:
      data: 1D array
      fs: sampling frequency (Hz)
      cutoff: cutoff frequency (Hz)
      order: filter order
    Returns filtered array same shape as data.
    """
    from scipy.signal import butter, filtfilt

    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


def rolling_mean_filter(data, window_samples):
    # Centered rolling mean with window_samples (odd recommended)
    s = pd.Series(data)
    return s.rolling(window=window_samples, center=True, min_periods=1).mean().to_numpy()


def estimate_sampling_frequency(time_array):
    # time_array in seconds (monotonic). Estimate fs from median dt
    dt = np.diff(time_array)
    # guard: remove zeros or negatives
    dt = dt[dt > 0]
    if len(dt) == 0:
        return 1.0
    median_dt = np.median(dt)
    if median_dt == 0:
        return 1.0
    return 1.0 / median_dt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', default='FT4_Primary.csv', help='CSV file name in Plotting/ (default FT4_Primary.csv)')
    parser.add_argument('--col', default='acceleration', help='Column to filter (default acceleration)')
    parser.add_argument('--cutoff', type=float, default=5.0, help='Low-pass cutoff frequency in Hz (used if SciPy available). Default 5 Hz')
    parser.add_argument('--order', type=int, default=4, help='Butterworth filter order (default 4)')
    parser.add_argument('--rolling-ms', type=float, default=200.0, help='Rolling window in milliseconds if SciPy not available. Default 200 ms')
    parser.add_argument('--output', default='Plots/lowpass_acceleration.png', help='Output PNG relative to Plotting/ folder')
    args = parser.parse_args()

    repo_plot_dir = Path(__file__).resolve().parent
    csv_path = repo_plot_dir / args.csv
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}. Try running from repository root or set --csv path relative to Plotting/")
        return

    df = pd.read_csv(csv_path, skipinitialspace=True)
    if args.col not in df.columns:
        print(f"Column '{args.col}' not in CSV. Available columns: {list(df.columns)[:10]} ...")
        return

    # Require time column
    if 'time' not in df.columns:
        print("No 'time' column in CSV. Aborting.")
        return

    t = df['time'].astype(float).to_numpy()
    x = df[args.col].astype(float).to_numpy()

    # estimate sampling frequency
    fs = estimate_sampling_frequency(t)
    print(f'Estimated sampling frequency: {fs:.2f} Hz')

    # try SciPy butterworth, fallback to rolling mean
    use_scipy = True
    try:
        filtered = butter_lowpass_filter(x, fs, args.cutoff, order=args.order)
    except Exception as e:
        print('SciPy filtfilt not available or failed, falling back to rolling mean. Reason:', e)
        use_scipy = False
        window_sec = args.rolling_ms / 1000.0
        window_samples = max(1, int(round(window_sec * fs)))
        if window_samples % 2 == 0:
            window_samples += 1
        filtered = rolling_mean_filter(x, window_samples)
        print(f'Rolling mean window samples: {window_samples}')

    # ensure output dir exists
    out_path = repo_plot_dir / args.output
    out_path.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(10, 6))
    plt.plot(t, x, label=f'raw {args.col}', alpha=0.5)
    plt.plot(t, filtered, label=f'lowpass {args.col} (cutoff={args.cutoff}Hz)' if use_scipy else f'rolling mean ({args.rolling_ms} ms)')
    plt.xlabel('time (s)')
    plt.ylabel(args.col)
    plt.legend()
    plt.title(f'Low-pass filter: {args.col}')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_path)
    print('Saved plot to', out_path)


if __name__ == '__main__':
    main()
