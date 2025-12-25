#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "sim_out.csv"

    df = pd.read_csv(csv_path)
    # basic sanity
    required = ["t_s","x_m","y_m","yaw_deg","v_mps","steer_deg","motor_nm","brake_pct"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise RuntimeError(f"Missing columns in {csv_path}: {missing}. Got: {list(df.columns)}")

    # 1) Trajectory
    plt.figure()
    plt.plot(df["x_m"], df["y_m"])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory (x vs y)")
    plt.axis("equal")
    plt.grid(True)

    # 2) Speed vs time
    plt.figure()
    plt.plot(df["t_s"], df["v_mps"])
    plt.xlabel("t (s)")
    plt.ylabel("v (m/s)")
    plt.title("Speed vs time")
    plt.grid(True)

    # 3) Steering & Yaw vs time
    plt.figure()
    plt.plot(df["t_s"], df["steer_deg"], label="steer_deg")
    plt.plot(df["t_s"], df["yaw_deg"], label="yaw_deg")
    plt.xlabel("t (s)")
    plt.ylabel("deg")
    plt.title("Steering & Yaw vs time")
    plt.legend()
    plt.grid(True)

    # 4) Inputs vs time
    plt.figure()
    plt.plot(df["t_s"], df["motor_nm"], label="motor_nm")
    plt.plot(df["t_s"], df["brake_pct"], label="brake_pct")
    plt.xlabel("t (s)")
    plt.title("Inputs vs time")
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()
