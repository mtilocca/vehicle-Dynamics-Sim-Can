#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "sim_out.csv"

    df = pd.read_csv(csv_path)
    # basic sanity
    required = ["t_s", "x_m", "y_m", "yaw_deg", "v_mps", "steer_deg", "motor_nm", "brake_pct", "batt_soc_pct", "batt_v", "batt_i"]
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

    # 5) Battery SOC vs time
    plt.figure()
    plt.plot(df["t_s"], df["batt_soc_pct"], label="SOC (%)", color='g')
    plt.xlabel("t (s)")
    plt.ylabel("SOC (%)")
    plt.title("Battery State of Charge (SOC) vs time")
    plt.grid(True)

    # 6) Battery Power vs time (Charge/Discharge)
    plt.figure()
    battery_power = df["batt_v"] * df["batt_i"] / 1000.0  # Power in kW (V * I / 1000)
    plt.plot(df["t_s"], battery_power, label="Battery Power (kW)", color='b')
    plt.xlabel("t (s)")
    plt.ylabel("Power (kW)")
    plt.title("Battery Power vs time")
    plt.grid(True)

    # 7) Motor Power vs time
    plt.figure()
    motor_power = df["motor_nm"] * df["v_mps"] / 1000.0  # Power in kW (Nm * m/s / 1000)
    plt.plot(df["t_s"], motor_power, label="Motor Power (kW)", color='r')
    plt.xlabel("t (s)")
    plt.ylabel("Power (kW)")
    plt.title("Motor Power vs time")
    plt.grid(True)

    # --- NEW: Regenerative braking power vs time ---
    # Calculate regen braking power (P_regen = V * I)
    df['regen_brake_kW'] = (df['batt_v'] * df['batt_i']) / 1000  # Convert to kW (W/1000)

    plt.figure()
    plt.plot(df["t_s"], df["regen_brake_kW"], label="Regenerative Braking Power (kW)", color='red')
    plt.xlabel("t (s)")
    plt.ylabel("Regenerative Power (kW)")
    plt.title("Regenerative Braking Power vs Time")
    plt.grid(True)
    plt.legend()
    # Show all plots
    plt.show()

if __name__ == "__main__":
    main()
