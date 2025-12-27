#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "sim_out.csv"

    df = pd.read_csv(csv_path)
    
    # Check if we have new format (truth/meas) or old format
    has_truth_meas = 'batt_soc_truth' in df.columns
    
    if has_truth_meas:
        # Use truth columns for plotting
        soc_col = 'batt_soc_truth'
        v_col = 'batt_v_truth'
        i_col = 'batt_i_truth'
    else:
        # Old format
        soc_col = 'batt_soc_pct'
        v_col = 'batt_v'
        i_col = 'batt_i'
    
    # Basic sanity check
    required = ["t_s", "x_m", "y_m", "yaw_deg", "v_mps", "steer_deg", "motor_nm", 
                "brake_pct", soc_col, v_col, i_col, "motor_power_kW", 
                "regen_power_kW", "brake_force_kN"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise RuntimeError(f"Missing columns in {csv_path}: {missing}. Got: {list(df.columns)}")

    if len(df) < 2:
        print(f"WARNING: Only {len(df)} rows in CSV. Run simulation longer!")
        print("The simulation may have crashed or only logged initialization.")
        return

    # Create figure with subplots (3x4 grid to include voltage)
    fig = plt.figure(figsize=(20, 10))
    
    # 1) Trajectory
    ax1 = plt.subplot(3, 4, 1)
    ax1.plot(df["x_m"], df["y_m"])
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Trajectory (x vs y)")
    ax1.axis("equal")
    ax1.grid(True)

    # 2) Speed vs time
    ax2 = plt.subplot(3, 4, 2)
    ax2.plot(df["t_s"], df["v_mps"])
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("v (m/s)")
    ax2.set_title("Speed vs time")
    ax2.grid(True)

    # 3) Steering & Yaw vs time
    ax3 = plt.subplot(3, 4, 3)
    ax3.plot(df["t_s"], df["steer_deg"], label="steer_deg")
    ax3.plot(df["t_s"], df["yaw_deg"], label="yaw_deg")
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("deg")
    ax3.set_title("Steering & Yaw vs time")
    ax3.legend()
    ax3.grid(True)

    # 4) Inputs vs time
    ax4 = plt.subplot(3, 4, 4)
    ax4.plot(df["t_s"], df["motor_nm"], label="motor_nm")
    ax4.plot(df["t_s"], df["brake_pct"], label="brake_pct")
    ax4.set_xlabel("t (s)")
    ax4.set_title("Inputs vs time")
    ax4.legend()
    ax4.grid(True)

    # 5) Battery SOC vs time
    ax5 = plt.subplot(3, 4, 5)
    ax5.plot(df["t_s"], df[soc_col], label="SOC (%) - Truth", color='g')
    if has_truth_meas:
        ax5.plot(df["t_s"], df["batt_soc_meas"], label="SOC (%) - Measured", 
                color='g', linestyle='--', alpha=0.7)
    ax5.set_xlabel("t (s)")
    ax5.set_ylabel("SOC (%)")
    ax5.set_title("Battery State of Charge (SOC) vs time")
    ax5.legend()
    ax5.grid(True)

    # 6) Battery Voltage vs time
    ax6 = plt.subplot(3, 4, 6)
    ax6.plot(df["t_s"], df[v_col], label="Voltage (V) - Truth", color='orange')
    if has_truth_meas:
        ax6.plot(df["t_s"], df["batt_v_meas"], label="Voltage (V) - Measured", 
                color='orange', linestyle='--', alpha=0.7)
    ax6.set_xlabel("t (s)")
    ax6.set_ylabel("Voltage (V)")
    ax6.set_title("Battery Voltage vs time")
    ax6.grid(True)
    ax6.legend()

    # 7) Battery Current vs time
    ax7 = plt.subplot(3, 4, 7)
    ax7.plot(df["t_s"], df[i_col], label="Battery Current (A) - Truth", color='purple')
    if has_truth_meas:
        ax7.plot(df["t_s"], df["batt_i_meas"], label="Current (A) - Measured", 
                color='purple', linestyle='--', alpha=0.7)
    ax7.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax7.set_xlabel("t (s)")
    ax7.set_ylabel("Current (A)")
    ax7.set_title("Battery Current vs time\n(+discharge, -charge)")
    ax7.legend()
    ax7.grid(True)

    # 8) Battery Power vs time (Combined Motor + Regen)
    ax8 = plt.subplot(3, 4, 8)
    net_battery_power = df["motor_power_kW"] - df["regen_power_kW"]
    
    ax8.plot(df["t_s"], net_battery_power, label="Net Battery Power (kW)", color='b')
    ax8.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax8.set_xlabel("t (s)")
    ax8.set_ylabel("Power (kW)")
    ax8.set_title("Net Battery Power vs time\n(+discharge, -charge)")
    ax8.legend()
    ax8.grid(True)

    # 9) Motor Power vs time
    ax9 = plt.subplot(3, 4, 9)
    ax9.plot(df["t_s"], df["motor_power_kW"], label="Motor Power (kW)", color='r')
    ax9.set_xlabel("t (s)")
    ax9.set_ylabel("Power (kW)")
    ax9.set_title("Motor Power vs time")
    ax9.legend()
    ax9.grid(True)

    # 10) Regenerative braking power vs time
    ax10 = plt.subplot(3, 4, 10)
    regen_active = df["regen_power_kW"] > 0.01
    if regen_active.any():
        ax10.fill_between(df["t_s"], 0, df["regen_power_kW"], 
                        where=regen_active, alpha=0.3, color='green', 
                        label='Regen Active')
    ax10.plot(df["t_s"], df["regen_power_kW"], label="Regen Power (kW)", color='green')
    ax10.set_xlabel("t (s)")
    ax10.set_ylabel("Regenerative Power (kW)")
    ax10.set_title("Regenerative Braking Power vs Time")
    ax10.set_ylim(bottom=0)
    ax10.legend()
    ax10.grid(True)

    # 11) SOC vs Voltage (characteristic curve)
    ax11 = plt.subplot(3, 4, 11)
    ax11.scatter(df[soc_col], df[v_col], c=df["t_s"], cmap='viridis', s=1, alpha=0.5)
    ax11.set_xlabel("SOC (%)")
    ax11.set_ylabel("Voltage (V)")
    ax11.set_title("Battery Characteristic Curve\n(SOC vs Voltage)")
    ax11.grid(True)
    cbar = plt.colorbar(ax11.collections[0], ax=ax11)
    cbar.set_label('Time (s)')

    # 12) Power vs Current (operating envelope)
    ax12 = plt.subplot(3, 4, 12)
    motor_active = df["motor_power_kW"] > 0.01
    regen_active_mask = df["regen_power_kW"] > 0.01
    
    if motor_active.any():
        ax12.scatter(df.loc[motor_active, i_col], 
                    df.loc[motor_active, "motor_power_kW"],
                    c='red', s=3, alpha=0.5, label='Motor (discharge)')
    
    if regen_active_mask.any():
        ax12.scatter(df.loc[regen_active_mask, i_col], 
                    -df.loc[regen_active_mask, "regen_power_kW"],
                    c='green', s=3, alpha=0.5, label='Regen (charge)')
    
    ax12.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax12.axvline(x=0, color='k', linestyle='--', linewidth=0.5)
    ax12.set_xlabel("Battery Current (A)")
    ax12.set_ylabel("Power (kW)")
    ax12.set_title("Battery Operating Envelope\n(Power vs Current)")
    ax12.legend()
    ax12.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()