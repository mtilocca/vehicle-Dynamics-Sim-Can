#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "sim_out.csv"

    df = pd.read_csv(csv_path)
    
    # Basic sanity check
    required = ["t_s", "x_m", "y_m", "yaw_deg", "v_mps", "steer_deg", "motor_nm", 
                "brake_pct", "batt_soc_pct", "batt_v", "batt_i", "motor_power_kW", 
                "regen_power_kW", "brake_force_kN"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise RuntimeError(f"Missing columns in {csv_path}: {missing}. Got: {list(df.columns)}")

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # 1) Trajectory
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(df["x_m"], df["y_m"])
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Trajectory (x vs y)")
    ax1.axis("equal")
    ax1.grid(True)

    # 2) Speed vs time
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(df["t_s"], df["v_mps"])
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("v (m/s)")
    ax2.set_title("Speed vs time")
    ax2.grid(True)

    # 3) Steering & Yaw vs time
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(df["t_s"], df["steer_deg"], label="steer_deg")
    ax3.plot(df["t_s"], df["yaw_deg"], label="yaw_deg")
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("deg")
    ax3.set_title("Steering & Yaw vs time")
    ax3.legend()
    ax3.grid(True)

    # 4) Inputs vs time
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(df["t_s"], df["motor_nm"], label="motor_nm")
    ax4.plot(df["t_s"], df["brake_pct"], label="brake_pct")
    ax4.set_xlabel("t (s)")
    ax4.set_title("Inputs vs time")
    ax4.legend()
    ax4.grid(True)

    # 5) Battery SOC vs time
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(df["t_s"], df["batt_soc_pct"], label="SOC (%)", color='g')
    ax5.set_xlabel("t (s)")
    ax5.set_ylabel("SOC (%)")
    ax5.set_title("Battery State of Charge (SOC) vs time")
    ax5.grid(True)

    # 6) Battery Power vs time (Combined Motor + Regen)
    ax6 = plt.subplot(3, 3, 6)
    # Motor power is positive when consuming (discharge)
    # Regen power is positive when recovering (charge)
    # Net battery power = motor_power - regen_power
    net_battery_power = df["motor_power_kW"] - df["regen_power_kW"]
    
    ax6.plot(df["t_s"], net_battery_power, label="Net Battery Power (kW)", color='b')
    ax6.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax6.set_xlabel("t (s)")
    ax6.set_ylabel("Power (kW)")
    ax6.set_title("Net Battery Power vs time\n(+discharge, -charge)")
    ax6.legend()
    ax6.grid(True)

    # 7) Motor Power vs time
    ax7 = plt.subplot(3, 3, 7)
    ax7.plot(df["t_s"], df["motor_power_kW"], label="Motor Power (kW)", color='r')
    ax7.set_xlabel("t (s)")
    ax7.set_ylabel("Power (kW)")
    ax7.set_title("Motor Power vs time")
    ax7.legend()
    ax7.grid(True)

    # 8) Regenerative braking power vs time (only show when > 0)
    ax8 = plt.subplot(3, 3, 8)
    # Only plot regen when it's actually happening
    regen_active = df["regen_power_kW"] > 0.01  # Small threshold to avoid noise
    if regen_active.any():
        ax8.fill_between(df["t_s"], 0, df["regen_power_kW"], 
                        where=regen_active, alpha=0.3, color='green', 
                        label='Regen Active')
    ax8.plot(df["t_s"], df["regen_power_kW"], label="Regen Power (kW)", color='green')
    ax8.set_xlabel("t (s)")
    ax8.set_ylabel("Regenerative Power (kW)")
    ax8.set_title("Regenerative Braking Power vs Time")
    ax8.set_ylim(bottom=0)  # Only show positive values
    ax8.legend()
    ax8.grid(True)

    # 9) Battery Current vs time
    ax9 = plt.subplot(3, 3, 9)
    ax9.plot(df["t_s"], df["batt_i"], label="Battery Current (A)", color='purple')
    ax9.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax9.set_xlabel("t (s)")
    ax9.set_ylabel("Current (A)")
    ax9.set_title("Battery Current vs time\n(+discharge, -charge)")
    ax9.legend()
    ax9.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()