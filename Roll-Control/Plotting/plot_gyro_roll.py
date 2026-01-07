#!/usr/bin/env python3
import argparse
from datetime import datetime, timezone
import pandas as pd
import matplotlib.pyplot as plt

def build_datetime(row):
    try:
        return datetime(
            int(row["year"]),
            int(row["month"]),
            int(row["day"]),
            int(row["hour"]),
            int(row["minute"]),
            int(row["second"]),
            tzinfo=timezone.utc,
        )
    except Exception:
        return pd.NaT

def main():
    ap = argparse.ArgumentParser(description="Plot gyro_roll vs time from rocket CSV, showing flight states")
    ap.add_argument("csv", help="Path to CSV file")
    ap.add_argument("-o", "--output", help="Path to save PNG (if omitted, shows window)")
    ap.add_argument("--use-datetime", action="store_true",
                    help="Use combined year/month/day/hour/minute/second as x-axis instead of the 'time' column")
    args = ap.parse_args()

    df = pd.read_csv(args.csv, skipinitialspace=True, engine="python")

    needed = ["gyro_roll", "state_name"]
    if args.use_datetime:   # ✅ fixed underscore
        needed += ["year", "month", "day", "hour", "minute", "second"]
    else:
        needed += ["time"]
    for col in needed:
        if col not in df.columns:
            raise SystemExit(f"Missing required column: {col}")

    # Prepare time axis
    if args.use_datetime:   # ✅ fixed underscore
        x = df.apply(build_datetime, axis=1)
        x_label = "Timestamp (UTC)"
    else:
        x = df["time"]
        x_label = "Time (s)"


    y = df["gyro_roll"]

     # Plot main line
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(x, y, color="black", label="Gyro Roll")

    # Add horizontal zero line
    ax.axhline(0, color="red", linestyle="--", linewidth=1, alpha=0.8, label="Zero Line")

    # Identify where state changes
    df["state_change"] = df["state_name"].ne(df["state_name"].shift())
    change_indices = df.index[df["state_change"]].tolist()
    change_indices.append(len(df) - 1)

    # Color palette for states
    colors = plt.cm.tab20.colors
    color_map = {}
    color_i = 0

    for i in range(len(change_indices) - 1):
        start = change_indices[i]
        end = change_indices[i + 1]
        state = df.loc[start, "state_name"]
        color = color_map.setdefault(state, colors[color_i % len(colors)])
        color_i += 1

        x_start = x.iloc[start]
        x_end = x.iloc[end]
        ax.axvspan(x_start, x_end, color=color, alpha=0.2)
        ax.text((x_start + (x_end - x_start) / 2), max(y)*0.95, state,
                ha="center", va="top", fontsize=8, color=color)

    ax.set_xlabel(x_label)
    ax.set_ylabel("Gyro Roll (°/s or rad/s)")
    ax.set_title("Gyro Roll vs Time with Flight States")
    ax.legend()
    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=150)
        print(f"Saved plot to {args.output}")
    else:
        # Print summary stats for main state
        if "state_name" in df.columns:
            main_data = df[df["state_name"].str.lower().str.contains("main", na=False)]
            if not main_data.empty:
                print("\n--- Main State Gyro Roll Summary ---")
                print(main_data["gyro_roll"].describe())
                mean_roll = main_data["gyro_roll"].mean()
                print(f"\nAverage Gyro Roll during MAIN: {mean_roll:.3f}")
            else:
                print("\nNo 'main' state found in data.")
        plt.show()

if __name__ == "__main__":
    main()
