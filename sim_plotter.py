#!/usr/bin/env python3
"""
sim_plotter.py - Vehicle Dynamics Simulation Plotter

Plots simulation results including:
  Figure 1: Vehicle dynamics, battery, and power
  Figure 2: Tire dynamics (Dugoff model outputs)
  Figure 3: 3-DOF lateral dynamics (if available)
  Figure 4: Wheel torque distribution (if available)
  Figure 5: Combined slip analysis (if available)

Usage:
  python3 sim_plotter.py [sim_out.csv] [--basic]

Options:
  --basic    Only plot Figures 1 and 2 (skip advanced 3-DOF plots)
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Import new plotting modules
try:
    from plot_utils import (
        get_battery_columns, has_3dof_data, has_tire_data,
        print_summary, validate_dataframe
    )
    from plot_3dof import (
        plot_lateral_dynamics, plot_wheel_torques, plot_combined_slip
    )
    USE_ADVANCED_PLOTS = True
except ImportError as e:
    print(f"[WARN] Advanced plotting modules not found: {e}")
    print("[WARN] Falling back to basic plots only")
    USE_ADVANCED_PLOTS = False


def plot_vehicle_dynamics(df, has_truth_meas, soc_col, v_col, i_col):
    """Figure 1: Vehicle dynamics, battery, and power"""

    fig = plt.figure(figsize=(20, 10))
    fig.suptitle("Figure 1: Vehicle Dynamics & Battery", fontsize=14, fontweight='bold')

    # 1) Trajectory — colored by direction, stop dots
    ax1 = plt.subplot(3, 4, 1)

    v = df["v_mps"].to_numpy()
    x = df["x_m"].to_numpy()
    y = df["y_m"].to_numpy()

    STOP_THRESH = 0.05  # m/s

    # Use gear_position if available (0=Neutral, 1=Forward, 2=Reverse)
    # direction array: 1=forward, -1=reverse, 0=neutral/stopped
    if "gear_position" in df.columns:
        gear = df["gear_position"].to_numpy()
        direction = np.where(gear == 1, 1, np.where(gear == 2, -1, 0))
    else:
        direction = np.where(v > STOP_THRESH, 1, np.where(v < -STOP_THRESH, -1, 0))

    _legend_shown = set()
    i = 0
    while i < len(direction):
        j = i + 1
        while j < len(direction) and direction[j] == direction[j - 1]:
            j += 1
        seg_dir = direction[i]
        if seg_dir == 1:
            color, lbl = "tab:green", "Forward"
        elif seg_dir == -1:
            color, lbl = "tab:red", "Reverse"
        else:
            color, lbl = "tab:gray", None
        kw = dict(color=color, linewidth=1.5)
        if lbl and lbl not in _legend_shown:
            kw["label"] = lbl
            _legend_shown.add(lbl)
        ax1.plot(x[i:j], y[i:j], **kw)
        i = j

    # Stop dots: transitions from moving → stopped (v crosses threshold)
    moving = np.abs(v) > STOP_THRESH
    stop_mask = np.zeros(len(v), dtype=bool)
    stop_mask[1:] = moving[:-1] & ~moving[1:]
    if stop_mask.any():
        ax1.scatter(x[stop_mask], y[stop_mask],
                    s=60, color="tab:orange", zorder=5, label="Stop", marker="o")

    # Start dot
    ax1.scatter(x[0], y[0], s=60, color="tab:blue", zorder=5, label="Start", marker="s")

    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Trajectory (x vs y)")
    ax1.axis("equal")
    ax1.legend(fontsize=7)
    ax1.grid(True)

    # 2) Speed & Acceleration vs time
    ax2 = plt.subplot(3, 4, 2)
    l_v, = ax2.plot(df["t_s"], df["v_mps"], label="velocity")
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("v (m/s)")
    ax2.set_title("Speed & Acceleration vs time")
    ax2.grid(True)

    if "a_long_mps2" in df.columns:
        ax2_twin = ax2.twinx()
        l_a, = ax2_twin.plot(df["t_s"], df["a_long_mps2"], 'r-', alpha=0.5, label="accel")
        ax2_twin.set_ylabel("a (m/s²)", color='r')
        ax2_twin.tick_params(axis='y', labelcolor='r')
        ax2_twin.axhline(y=0, color='r', linestyle='--', linewidth=0.5, alpha=0.6)

        # --- KEY FIX: symmetric accel axis around 0 so braking is visible ---
        a_min = float(df["a_long_mps2"].min())
        a_max = float(df["a_long_mps2"].max())
        a_abs = max(abs(a_min), abs(a_max))
        if a_abs > 1e-9:
            ax2_twin.set_ylim(-1.1 * a_abs, 1.1 * a_abs)

        # combined legend
        ax2.legend(handles=[l_v, l_a], labels=["velocity", "accel"], loc="best")

    # 3) Steering: requested vs achieved (+ wheel angles if available) + Yaw on twin axis
    ax3 = plt.subplot(3, 4, 3)

    # --- steering signals (left axis)
    handles = []
    labels = []

    # Requested steer (if logged)
    if "steer_cmd_deg" in df.columns:
        l_cmd, = ax3.plot(df["t_s"], df["steer_cmd_deg"], linestyle="--", alpha=0.8)
        handles.append(l_cmd); labels.append("steer_cmd_deg")

    # Achieved/actual steer (your existing steer_deg)
    if "steer_deg" in df.columns:
        l_act, = ax3.plot(df["t_s"], df["steer_deg"])
        handles.append(l_act); labels.append("steer_deg")

    # Front wheel angles if present (often fl_deg/fr_deg)
    if "delta_fl_deg" in df.columns:
        l_fl, = ax3.plot(df["t_s"], df["delta_fl_deg"], alpha=0.6)
        handles.append(l_fl); labels.append("fl_deg")

    if "delta_fr_deg" in df.columns:
        l_fr, = ax3.plot(df["t_s"], df["delta_fr_deg"], alpha=0.6)
        handles.append(l_fr); labels.append("fr_deg")

    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("Steering (deg)")
    ax3.set_title("Steering (cmd vs actual) & Yaw vs time")
    ax3.grid(True)

    # --- yaw on right axis
    ax3_yaw = ax3.twinx()
    if "yaw_deg" in df.columns:
        l_yaw, = ax3_yaw.plot(df["t_s"], df["yaw_deg"], color="orange", alpha=0.8)
        ax3_yaw.set_ylabel("Yaw (deg)")
        handles.append(l_yaw); labels.append("yaw_deg")

    # Combined legend
    if handles:
        ax3.legend(handles, labels, loc="best")


    # 4) Inputs vs time (Torque + Brake Force + Brake Pedal %)
    ax4 = plt.subplot(3, 4, 4)

    # Primary axis: motor torque
    l_m, = ax4.plot(df["t_s"], df["motor_nm"], label="motor_nm")
    ax4.set_xlabel("t (s)")
    ax4.set_ylabel("Motor torque (Nm)")
    ax4.set_title("Inputs vs time")
    ax4.grid(True)

    # Secondary axis (right): brake force in kN
    ax4_twin_force = ax4.twinx()
    l_bf, = ax4_twin_force.plot(df["t_s"], df["brake_force_kN"], label="brake_force_kN", alpha=0.8, color='green')
    ax4_twin_force.set_ylabel("Brake force (kN)")

    # Tertiary axis (offset right): brake pedal in %
    ax4_twin_pct = ax4.twinx()
    ax4_twin_pct.spines["right"].set_position(("axes", 1.12))
    l_bp, = ax4_twin_pct.plot(df["t_s"], df["brake_pct"], linestyle="--", alpha=0.7, label="brake_pct", color='red')
    ax4_twin_pct.set_ylabel("Brake pedal (%)")
    ax4_twin_pct.set_ylim(0, 100)

    # Combined legend for the three axes
    ax4.legend(handles=[l_m, l_bf, l_bp],
              labels=["motor_nm", "brake_force_kN", "brake_pct"],
              loc="best")

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
    return fig


def plot_tire_dynamics(df):
    """Figure 2: Tire dynamics (Dugoff model outputs)"""

    # Check if tire dynamics columns exist
    tire_cols = ["Fx_fl", "Fx_fr", "Fx_rl", "Fx_rr",
                 "Fy_fl", "Fy_fr", "Fy_rl", "Fy_rr",
                 "Fz_fl", "Fz_fr", "Fz_rl", "Fz_rr",
                 "sigma_x_fl", "sigma_x_fr", "sigma_x_rl", "sigma_x_rr",
                 "sigma_y_fl", "sigma_y_fr", "sigma_y_rl", "sigma_y_rr",
                 "lambda_fl", "lambda_fr", "lambda_rl", "lambda_rr",
                 "surface_mu"]

    missing = [c for c in tire_cols if c not in df.columns]
    if missing:
        print(f"[INFO] Tire dynamics columns not found: {missing[:5]}...")
        print("[INFO] Run with --dynamic-model to enable tire dynamics logging")
        return None

    # Check if dynamic model was enabled
    if "dynamic_model" in df.columns:
        if df["dynamic_model"].iloc[0] == 0:
            print("[INFO] Dynamic model was DISABLED - tire forces are zeros")

    fig = plt.figure(figsize=(20, 12))
    fig.suptitle("Figure 2: Tire Dynamics (Dugoff Model)", fontsize=14, fontweight='bold')

    t = df["t_s"]

    # ========================================================================
    # Row 1: Forces
    # ========================================================================

    # 1) Longitudinal Forces (Fx) - All 4 wheels
    ax1 = plt.subplot(3, 4, 1)
    ax1.plot(t, df["Fx_fl"] / 1000, label="FL", alpha=0.8)
    ax1.plot(t, df["Fx_fr"] / 1000, label="FR", alpha=0.8)
    ax1.plot(t, df["Fx_rl"] / 1000, label="RL", linewidth=2)
    ax1.plot(t, df["Fx_rr"] / 1000, label="RR", linewidth=2)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("Fx (kN)")
    ax1.set_title("Longitudinal Tire Forces\n(+drive, -brake)")
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True)

    # 2) Total Longitudinal Force
    ax2 = plt.subplot(3, 4, 2)
    Fx_total = df["Fx_fl"] + df["Fx_fr"] + df["Fx_rl"] + df["Fx_rr"]
    Fx_rear = df["Fx_rl"] + df["Fx_rr"]
    ax2.plot(t, Fx_total / 1000, label="Fx_total", color='blue', linewidth=2)
    ax2.plot(t, Fx_rear / 1000, label="Fx_rear (driven)", color='red', alpha=0.7)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("Fx (kN)")
    ax2.set_title("Total Longitudinal Force")
    ax2.legend()
    ax2.grid(True)

    # 3) Lateral Forces (Fy) - All 4 wheels
    ax3 = plt.subplot(3, 4, 3)
    ax3.plot(t, df["Fy_fl"] / 1000, label="FL", alpha=0.8)
    ax3.plot(t, df["Fy_fr"] / 1000, label="FR", alpha=0.8)
    ax3.plot(t, df["Fy_rl"] / 1000, label="RL", alpha=0.8)
    ax3.plot(t, df["Fy_rr"] / 1000, label="RR", alpha=0.8)
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("Fy (kN)")
    ax3.set_title("Lateral Tire Forces\n(+left, -right)")
    ax3.legend(loc='upper right', fontsize=8)
    ax3.grid(True)

    # 4) Normal Loads (Fz) - All 4 wheels
    ax4 = plt.subplot(3, 4, 4)
    ax4.plot(t, df["Fz_fl"] / 1000, label="FL")
    ax4.plot(t, df["Fz_fr"] / 1000, label="FR")
    ax4.plot(t, df["Fz_rl"] / 1000, label="RL")
    ax4.plot(t, df["Fz_rr"] / 1000, label="RR")
    ax4.set_xlabel("t (s)")
    ax4.set_ylabel("Fz (kN)")
    ax4.set_title("Normal Loads (Weight Distribution)")
    ax4.legend(loc='upper right', fontsize=8)
    ax4.grid(True)

    # ========================================================================
    # Row 2: Slip Ratios
    # ========================================================================

    # 5) Longitudinal Slip (sigma_x) - All 4 wheels
    ax5 = plt.subplot(3, 4, 5)
    ax5.plot(t, df["sigma_x_fl"], label="FL", alpha=0.8)
    ax5.plot(t, df["sigma_x_fr"], label="FR", alpha=0.8)
    ax5.plot(t, df["sigma_x_rl"], label="RL", linewidth=2)
    ax5.plot(t, df["sigma_x_rr"], label="RR", linewidth=2)
    ax5.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax5.axhline(y=0.1, color='r', linestyle=':', linewidth=0.5, label='peak slip')
    ax5.axhline(y=-0.1, color='r', linestyle=':', linewidth=0.5)
    ax5.set_xlabel("t (s)")
    ax5.set_ylabel("σx (dimensionless)")
    ax5.set_title("Longitudinal Slip Ratio\n(+accel, -brake)")
    ax5.legend(loc='upper right', fontsize=8)
    ax5.grid(True)

    # 6) Lateral Slip (sigma_y) - All 4 wheels
    ax6 = plt.subplot(3, 4, 6)
    ax6.plot(t, df["sigma_y_fl"], label="FL", alpha=0.8)
    ax6.plot(t, df["sigma_y_fr"], label="FR", alpha=0.8)
    ax6.plot(t, df["sigma_y_rl"], label="RL", alpha=0.8)
    ax6.plot(t, df["sigma_y_rr"], label="RR", alpha=0.8)
    ax6.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax6.set_xlabel("t (s)")
    ax6.set_ylabel("σy (dimensionless)")
    ax6.set_title("Lateral Slip Ratio\n(≈ slip angle)")
    ax6.legend(loc='upper right', fontsize=8)
    ax6.grid(True)

    # 7) Friction Utilization (lambda) - All 4 wheels
    ax7 = plt.subplot(3, 4, 7)
    ax7.plot(t, df["lambda_fl"], label="FL", alpha=0.8)
    ax7.plot(t, df["lambda_fr"], label="FR", alpha=0.8)
    ax7.plot(t, df["lambda_rl"], label="RL", linewidth=2)
    ax7.plot(t, df["lambda_rr"], label="RR", linewidth=2)
    ax7.axhline(y=1.0, color='r', linestyle='--', linewidth=1, label='saturation')
    ax7.set_xlabel("t (s)")
    ax7.set_ylabel("λ (dimensionless)")
    ax7.set_title("Friction Utilization\n(λ<1 = saturated)")
    ax7.legend(loc='upper right', fontsize=8)
    ax7.grid(True)
    ax7.set_ylim(0, max(2.0, df[["lambda_fl", "lambda_fr", "lambda_rl", "lambda_rr"]].max().max() * 1.1))

    # 8) Surface Friction
    ax8 = plt.subplot(3, 4, 8)
    ax8.plot(t, df["surface_mu"], label="μ", color='brown', linewidth=2)
    ax8.axhline(y=0.85, color='g', linestyle=':', label='dry pavement')
    ax8.axhline(y=0.72, color='orange', linestyle=':', label='compact gravel')
    ax8.axhline(y=0.30, color='r', linestyle=':', label='wet dust')
    ax8.set_xlabel("t (s)")
    ax8.set_ylabel("μ (dimensionless)")
    ax8.set_title("Surface Friction Coefficient")
    ax8.legend(loc='upper right', fontsize=8)
    ax8.grid(True)
    ax8.set_ylim(0, 1.0)

    # ========================================================================
    # Row 3: Combined Analysis
    # ========================================================================

    # 9) Friction Circle - Rear Axle
    ax9 = plt.subplot(3, 4, 9)
    Fx_r = (df["Fx_rl"] + df["Fx_rr"]) / 1000
    Fy_r = (df["Fy_rl"] + df["Fy_rr"]) / 1000
    Fz_r = (df["Fz_rl"] + df["Fz_rr"]) / 1000
    mu = df["surface_mu"].mean()

    theta = np.linspace(0, 2*np.pi, 100)
    F_max = mu * Fz_r.mean()
    ax9.plot(F_max * np.cos(theta), F_max * np.sin(theta), 'r--', label=f'μ={mu:.2f} limit')

    scatter = ax9.scatter(Fy_r, Fx_r, c=t, cmap='viridis', s=5, alpha=0.5)
    ax9.set_xlabel("Fy_rear (kN)")
    ax9.set_ylabel("Fx_rear (kN)")
    ax9.set_title("Friction Circle - Rear Axle")
    ax9.axis('equal')
    ax9.legend(loc='upper right', fontsize=8)
    ax9.grid(True)
    plt.colorbar(scatter, ax=ax9, label='Time (s)')

    # 10) Friction Circle - Front Axle
    ax10 = plt.subplot(3, 4, 10)
    Fx_f = (df["Fx_fl"] + df["Fx_fr"]) / 1000
    Fy_f = (df["Fy_fl"] + df["Fy_fr"]) / 1000
    Fz_f = (df["Fz_fl"] + df["Fz_fr"]) / 1000

    F_max_f = mu * Fz_f.mean()
    ax10.plot(F_max_f * np.cos(theta), F_max_f * np.sin(theta), 'r--', label=f'μ={mu:.2f} limit')

    scatter2 = ax10.scatter(Fy_f, Fx_f, c=t, cmap='viridis', s=5, alpha=0.5)
    ax10.set_xlabel("Fy_front (kN)")
    ax10.set_ylabel("Fx_front (kN)")
    ax10.set_title("Friction Circle - Front Axle")
    ax10.axis('equal')
    ax10.legend(loc='upper right', fontsize=8)
    ax10.grid(True)
    plt.colorbar(scatter2, ax=ax10, label='Time (s)')

    # 11) Load Transfer vs Acceleration
    ax11 = plt.subplot(3, 4, 11)
    if "a_long_mps2" in df.columns:
        Fz_front = df["Fz_fl"] + df["Fz_fr"]
        Fz_rear = df["Fz_rl"] + df["Fz_rr"]
        load_ratio = Fz_rear / (Fz_front + Fz_rear) * 100

        ax11.scatter(df["a_long_mps2"], load_ratio, c=t, cmap='viridis', s=5, alpha=0.5)
        ax11.axhline(y=50, color='k', linestyle='--', linewidth=0.5, label='static')
        ax11.set_xlabel("Longitudinal Accel (m/s²)")
        ax11.set_ylabel("Rear Axle Load (%)")
        ax11.set_title("Load Transfer\n(accel→rear, brake→front)")
        ax11.legend()
        ax11.grid(True)
    else:
        ax11.text(0.5, 0.5, "a_long_mps2 not in CSV", ha='center', va='center')
        ax11.set_title("Load Transfer (no data)")

    # 12) Slip vs Force (Dugoff characteristic)
    ax12 = plt.subplot(3, 4, 12)
    sigma_x = df["sigma_x_rl"]
    Fx_rl = df["Fx_rl"] / 1000

    scatter3 = ax12.scatter(sigma_x, Fx_rl, c=t, cmap='viridis', s=5, alpha=0.5, label='actual')

    Fz_avg = df["Fz_rl"].mean()
    mu_avg = df["surface_mu"].mean()
    ax12.axhline(y=mu_avg * Fz_avg / 1000, color='r', linestyle='--', label=f'μFz limit')
    ax12.axhline(y=-mu_avg * Fz_avg / 1000, color='r', linestyle='--')

    ax12.set_xlabel("σx (slip ratio)")
    ax12.set_ylabel("Fx (kN)")
    ax12.set_title("Slip-Force Characteristic\n(Rear-Left Wheel)")
    ax12.legend(loc='upper right', fontsize=8)
    ax12.grid(True)
    ax12.set_xlim(-0.35, 0.35)
    plt.colorbar(scatter3, ax=ax12, label='Time (s)')

    plt.tight_layout()
    return fig


def main():
    # Parse command line arguments
    csv_path = "sim_out.csv"
    basic_only = False

    for arg in sys.argv[1:]:
        if arg == "--basic":
            basic_only = True
        elif not arg.startswith("--"):
            csv_path = arg

    print(f"[INFO] Loading: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"[INFO] Loaded {len(df)} rows, {len(df.columns)} columns")

    # Determine battery column naming
    if USE_ADVANCED_PLOTS:
        has_truth_meas, soc_col, v_col, i_col = get_battery_columns(df)
        # Validate dataframe
        if not validate_dataframe(df, csv_path):
            return
        # Print comprehensive summary
        print_summary(df)
    else:
        # Fallback for basic mode
        has_truth_meas = 'batt_soc_truth' in df.columns
        if has_truth_meas:
            soc_col = 'batt_soc_truth'
            v_col = 'batt_v_truth'
            i_col = 'batt_i_truth'
        else:
            soc_col = 'batt_soc_pct'
            v_col = 'batt_v'
            i_col = 'batt_i'

        required = ["t_s", "x_m", "y_m", "yaw_deg", "v_mps", "steer_deg", "motor_nm",
                    "brake_pct", soc_col, v_col, i_col, "motor_power_kW",
                    "regen_power_kW", "brake_force_kN"]
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise RuntimeError(f"Missing columns in {csv_path}: {missing}.\nGot: {list(df.columns)}")

        if len(df) < 2:
            print(f"WARNING: Only {len(df)} rows in CSV. Run simulation longer!")
            return

        # Basic summary
        print(f"\n{'='*60}")
        print("SIMULATION SUMMARY")
        print(f"{'='*60}")
        print(f"Duration:     {df['t_s'].max():.1f} s")
        print(f"Distance:     {np.sqrt(df['x_m'].iloc[-1]**2 + df['y_m'].iloc[-1]**2):.1f} m")
        print(f"Max Speed:    {df['v_mps'].max():.1f} m/s ({df['v_mps'].max() * 3.6:.1f} km/h)")
        print(f"Final Speed:  {df['v_mps'].iloc[-1]:.1f} m/s")
        print(f"SOC Change:   {df[soc_col].iloc[0]:.1f}% → {df[soc_col].iloc[-1]:.1f}%")
        print(f"{'='*60}\n")

    # Generate Figure 1: Vehicle Dynamics & Battery
    print("[INFO] Generating Figure 1: Vehicle Dynamics & Battery")
    fig1 = plot_vehicle_dynamics(df, has_truth_meas, soc_col, v_col, i_col)

    # Generate Figure 2: Tire Dynamics (if available)
    tire_data_available = "Fx_rl" in df.columns
    if tire_data_available:
        print("[INFO] Generating Figure 2: Tire Dynamics")
        fig2 = plot_tire_dynamics(df)
    else:
        print("[INFO] Skipping Figure 2 (tire data not available)")

    # Generate advanced 3-DOF plots if modules are available and not in basic mode
    if USE_ADVANCED_PLOTS and not basic_only:
        # Figure 3: Lateral Dynamics
        if has_3dof_data(df):
            print("[INFO] Generating Figure 3: 3-DOF Lateral Dynamics")
            fig3 = plot_lateral_dynamics(df)
        else:
            print("[INFO] Skipping Figure 3 (3-DOF data not available)")

        # Figure 4: Wheel Torques
        if "tau_drive_rl_nm" in df.columns:
            print("[INFO] Generating Figure 4: Wheel Torque Distribution")
            fig4 = plot_wheel_torques(df)
        else:
            print("[INFO] Skipping Figure 4 (torque data not available)")

        # Figure 5: Combined Slip
        if has_tire_data(df):
            print("[INFO] Generating Figure 5: Combined Slip Analysis")
            fig5 = plot_combined_slip(df)
        else:
            print("[INFO] Skipping Figure 5 (slip data not available)")
    elif basic_only:
        print("[INFO] Basic mode: skipping advanced 3-DOF plots")

    print("\n[INFO] All plots generated. Showing...")
    plt.show()


if __name__ == "__main__":
    main()
