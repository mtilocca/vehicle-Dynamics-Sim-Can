#!/usr/bin/env python3
"""
plot_3dof.py - Advanced 3-DOF Vehicle Dynamics Visualization

Specialized plots for 3-DOF (longitudinal, lateral, yaw) dynamics including:
  - Lateral velocity and slip angles
  - Yaw rate response and understeer analysis
  - Wheel torque distribution
  - Combined slip analysis
  - Phase portraits and stability diagrams
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from plot_utils import (
    check_columns, format_time_axis, add_zero_line, symmetric_limits,
    compute_sideslip_angle, compute_total_slip, deg_to_rad, rad_to_deg
)


def plot_lateral_dynamics(df):
    """
    Figure 3: Lateral Dynamics (vy, yaw_rate, slip angles, lateral acceleration)
    """
    # Check required columns
    required = ["t_s", "vy_mps", "yaw_rate_radps", "a_lat_mps2"]
    has_all, missing = check_columns(df, required)
    if not has_all:
        print(f"[WARN] Missing columns for lateral dynamics: {missing}")
        return None

    fig = plt.figure(figsize=(20, 12))
    fig.suptitle("Figure 3: 3-DOF Lateral Dynamics", fontsize=14, fontweight='bold')

    t = df["t_s"]

    # ========================================================================
    # Row 1: Basic Lateral States
    # ========================================================================

    # 1) Lateral Velocity (vy)
    ax1 = plt.subplot(3, 4, 1)
    ax1.plot(t, df["vy_mps"], linewidth=2, color='blue')
    add_zero_line(ax1)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("Lateral Velocity (m/s)")
    ax1.set_title("Lateral Velocity (vy)\n(+left, -right)")
    ax1.grid(True)
    symmetric_limits(ax1, df["vy_mps"])

    # 2) Yaw Rate
    ax2 = plt.subplot(3, 4, 2)
    yaw_rate_deg = rad_to_deg(df["yaw_rate_radps"])
    ax2.plot(t, yaw_rate_deg, linewidth=2, color='orange')
    add_zero_line(ax2)
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("Yaw Rate (deg/s)")
    ax2.set_title("Yaw Rate\n(+CCW, -CW)")
    ax2.grid(True)
    symmetric_limits(ax2, yaw_rate_deg)

    # 3) Lateral Acceleration
    ax3 = plt.subplot(3, 4, 3)
    ax3.plot(t, df["a_lat_mps2"], linewidth=2, color='red')
    ax3.plot(t, df["a_lat_mps2"] / 9.81, linewidth=1, alpha=0.5, label='g-force')
    add_zero_line(ax3)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("Lateral Accel (m/s²)")
    ax3.set_title("Lateral Acceleration\n(+left, -right)")
    ax3.legend()
    ax3.grid(True)
    symmetric_limits(ax3, df["a_lat_mps2"])

    # 4) Sideslip Angle (β = atan2(vy, vx))
    ax4 = plt.subplot(3, 4, 4)
    beta_deg = compute_sideslip_angle(df)
    if beta_deg is not None:
        ax4.plot(t, beta_deg, linewidth=2, color='purple')
        add_zero_line(ax4)
        ax4.axhline(y=3, color='orange', linestyle=':', linewidth=1, label='typical limit')
        ax4.axhline(y=-3, color='orange', linestyle=':', linewidth=1)
        ax4.set_xlabel("t (s)")
        ax4.set_ylabel("Sideslip Angle β (deg)")
        ax4.set_title("Vehicle Sideslip Angle\n(β = atan2(vy, vx))")
        ax4.legend()
        ax4.grid(True)
        symmetric_limits(ax4, beta_deg)
    else:
        ax4.text(0.5, 0.5, "Sideslip data unavailable", ha='center', va='center')

    # ========================================================================
    # Row 2: Slip Angles (tire-level)
    # ========================================================================

    # 5) Front Slip Angles
    ax5 = plt.subplot(3, 4, 5)
    if "alpha_fl" in df.columns and "alpha_fr" in df.columns:
        ax5.plot(t, rad_to_deg(df["alpha_fl"]), label="FL", alpha=0.8)
        ax5.plot(t, rad_to_deg(df["alpha_fr"]), label="FR", alpha=0.8)
        add_zero_line(ax5)
        ax5.set_xlabel("t (s)")
        ax5.set_ylabel("Slip Angle (deg)")
        ax5.set_title("Front Slip Angles\n(α = tire slip angle)")
        ax5.legend()
        ax5.grid(True)
    else:
        ax5.text(0.5, 0.5, "Front slip angles unavailable", ha='center', va='center')

    # 6) Rear Slip Angles
    ax6 = plt.subplot(3, 4, 6)
    if "alpha_rl" in df.columns and "alpha_rr" in df.columns:
        ax6.plot(t, rad_to_deg(df["alpha_rl"]), label="RL", alpha=0.8)
        ax6.plot(t, rad_to_deg(df["alpha_rr"]), label="RR", alpha=0.8)
        add_zero_line(ax6)
        ax6.set_xlabel("t (s)")
        ax6.set_ylabel("Slip Angle (deg)")
        ax6.set_title("Rear Slip Angles\n(α = tire slip angle)")
        ax6.legend()
        ax6.grid(True)
    else:
        ax6.text(0.5, 0.5, "Rear slip angles unavailable", ha='center', va='center')

    # 7) All Slip Angles Combined
    ax7 = plt.subplot(3, 4, 7)
    if all(col in df.columns for col in ["alpha_fl", "alpha_fr", "alpha_rl", "alpha_rr"]):
        ax7.plot(t, rad_to_deg(df["alpha_fl"]), label="FL", alpha=0.6, linewidth=1)
        ax7.plot(t, rad_to_deg(df["alpha_fr"]), label="FR", alpha=0.6, linewidth=1)
        ax7.plot(t, rad_to_deg(df["alpha_rl"]), label="RL", alpha=0.8, linewidth=2)
        ax7.plot(t, rad_to_deg(df["alpha_rr"]), label="RR", alpha=0.8, linewidth=2)
        add_zero_line(ax7)
        ax7.set_xlabel("t (s)")
        ax7.set_ylabel("Slip Angle (deg)")
        ax7.set_title("All Tire Slip Angles")
        ax7.legend(loc='upper right', fontsize=8)
        ax7.grid(True)
    else:
        ax7.text(0.5, 0.5, "Slip angle data unavailable", ha='center', va='center')

    # 8) Lateral Velocity vs Yaw Rate (Phase Portrait)
    ax8 = plt.subplot(3, 4, 8)
    scatter = ax8.scatter(yaw_rate_deg, df["vy_mps"], c=t, cmap='viridis', s=3, alpha=0.6)
    add_zero_line(ax8, axis='both')
    ax8.set_xlabel("Yaw Rate (deg/s)")
    ax8.set_ylabel("Lateral Velocity (m/s)")
    ax8.set_title("Lateral Phase Portrait\n(vy vs yaw_rate)")
    ax8.grid(True)
    plt.colorbar(scatter, ax=ax8, label='Time (s)')

    # ========================================================================
    # Row 3: Response and Coupling
    # ========================================================================

    # 9) Steering vs Yaw Rate Response
    ax9 = plt.subplot(3, 4, 9)
    if "steer_deg" in df.columns:
        ax9_steer = ax9.twinx()
        l1, = ax9.plot(t, yaw_rate_deg, label="Yaw Rate", color='blue', linewidth=2)
        l2, = ax9_steer.plot(t, df["steer_deg"], label="Steering", color='green', alpha=0.6, linestyle='--')
        add_zero_line(ax9)
        ax9.set_xlabel("t (s)")
        ax9.set_ylabel("Yaw Rate (deg/s)", color='blue')
        ax9_steer.set_ylabel("Steering (deg)", color='green')
        ax9.set_title("Yaw Rate Response to Steering")
        ax9.legend(handles=[l1, l2], loc='upper right')
        ax9.grid(True)
    else:
        ax9.text(0.5, 0.5, "Steering data unavailable", ha='center', va='center')

    # 10) Lateral Accel vs Steering (Understeer Diagram)
    ax10 = plt.subplot(3, 4, 10)
    if "steer_deg" in df.columns:
        # Filter for non-zero steering
        mask = np.abs(df["steer_deg"]) > 0.5
        if mask.any():
            scatter2 = ax10.scatter(df.loc[mask, "steer_deg"],
                                   df.loc[mask, "a_lat_mps2"],
                                   c=df.loc[mask, "v_mps"],
                                   cmap='plasma', s=5, alpha=0.5)
            add_zero_line(ax10)
            ax10.set_xlabel("Steering Angle (deg)")
            ax10.set_ylabel("Lateral Accel (m/s²)")
            ax10.set_title("Understeer Analysis\n(a_lat vs δ)")
            ax10.grid(True)
            plt.colorbar(scatter2, ax=ax10, label='Speed (m/s)')
        else:
            ax10.text(0.5, 0.5, "No steering input", ha='center', va='center')
    else:
        ax10.text(0.5, 0.5, "Steering data unavailable", ha='center', va='center')

    # 11) Yaw Rate vs Lateral Accel
    ax11 = plt.subplot(3, 4, 11)
    scatter3 = ax11.scatter(df["a_lat_mps2"], yaw_rate_deg, c=t, cmap='viridis', s=3, alpha=0.6)
    add_zero_line(ax11, axis='both')
    ax11.set_xlabel("Lateral Accel (m/s²)")
    ax11.set_ylabel("Yaw Rate (deg/s)")
    ax11.set_title("Lateral Accel vs Yaw Rate")
    ax11.grid(True)
    plt.colorbar(scatter3, ax=ax11, label='Time (s)')

    # 12) Trajectory with Velocity Vectors
    ax12 = plt.subplot(3, 4, 12)
    # Subsample for clarity
    step = max(1, len(df) // 50)
    idx = range(0, len(df), step)

    ax12.plot(df["x_m"], df["y_m"], 'b-', linewidth=1, alpha=0.5, label='path')

    # Velocity vectors (vx, vy) in global frame
    for i in idx:
        x = df["x_m"].iloc[i]
        y = df["y_m"].iloc[i]
        yaw = deg_to_rad(df["yaw_deg"].iloc[i])
        vx_body = df["v_mps"].iloc[i]  # Approximation
        vy_body = df["vy_mps"].iloc[i]

        # Rotate to global frame
        vx_global = vx_body * np.cos(yaw) - vy_body * np.sin(yaw)
        vy_global = vx_body * np.sin(yaw) + vy_body * np.cos(yaw)

        scale = 0.5  # Arrow scaling
        ax12.arrow(x, y, vx_global * scale, vy_global * scale,
                  head_width=0.3, head_length=0.2, fc='red', ec='red', alpha=0.6)

    ax12.set_xlabel("x (m)")
    ax12.set_ylabel("y (m)")
    ax12.set_title("Trajectory with Velocity Vectors\n(red = velocity direction)")
    ax12.axis("equal")
    ax12.grid(True)
    ax12.legend()

    plt.tight_layout()
    return fig


def plot_wheel_torques(df):
    """
    Figure 4: Wheel Torque Distribution and Drive Dynamics
    """
    # Check required columns
    torque_cols = ["tau_drive_rl_nm", "tau_drive_rr_nm",
                   "tau_brake_fl_nm", "tau_brake_fr_nm",
                   "tau_brake_rl_nm", "tau_brake_rr_nm"]
    has_all, missing = check_columns(df, torque_cols)
    if not has_all:
        print(f"[WARN] Missing torque columns: {missing}")
        return None

    fig = plt.figure(figsize=(20, 10))
    fig.suptitle("Figure 4: Wheel Torque Distribution", fontsize=14, fontweight='bold')

    t = df["t_s"]

    # ========================================================================
    # Row 1: Drive Torques
    # ========================================================================

    # 1) Drive Torques - Rear Wheels
    ax1 = plt.subplot(3, 4, 1)
    ax1.plot(t, df["tau_drive_rl_nm"], label="RL", linewidth=2)
    ax1.plot(t, df["tau_drive_rr_nm"], label="RR", linewidth=2)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("Torque (Nm)")
    ax1.set_title("Drive Torques - Rear Axle")
    ax1.legend()
    ax1.grid(True)

    # 2) Total Drive Torque
    ax2 = plt.subplot(3, 4, 2)
    tau_drive_total = df["tau_drive_rl_nm"] + df["tau_drive_rr_nm"]
    ax2.plot(t, tau_drive_total, linewidth=2, color='green')
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("Total Torque (Nm)")
    ax2.set_title("Total Drive Torque (RL + RR)")
    ax2.grid(True)

    # 3) Drive Torque Asymmetry
    ax3 = plt.subplot(3, 4, 3)
    tau_drive_diff = df["tau_drive_rr_nm"] - df["tau_drive_rl_nm"]
    ax3.plot(t, tau_drive_diff, linewidth=2, color='orange')
    add_zero_line(ax3)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("Torque Difference (Nm)")
    ax3.set_title("Drive Torque Asymmetry\n(RR - RL)")
    ax3.grid(True)
    symmetric_limits(ax3, tau_drive_diff)

    # 4) Motor Torque vs Total Drive Torque
    ax4 = plt.subplot(3, 4, 4)
    if "motor_nm" in df.columns:
        ax4.plot(t, df["motor_nm"], label="Motor Command", linestyle='--', alpha=0.7)
        ax4.plot(t, tau_drive_total, label="Total Drive Torque", linewidth=2)
        ax4.set_xlabel("t (s)")
        ax4.set_ylabel("Torque (Nm)")
        ax4.set_title("Motor Cmd vs Actual Drive Torque")
        ax4.legend()
        ax4.grid(True)

    # ========================================================================
    # Row 2: Brake Torques
    # ========================================================================

    # 5) Brake Torques - Front Wheels
    ax5 = plt.subplot(3, 4, 5)
    ax5.plot(t, df["tau_brake_fl_nm"], label="FL", alpha=0.8)
    ax5.plot(t, df["tau_brake_fr_nm"], label="FR", alpha=0.8)
    ax5.set_xlabel("t (s)")
    ax5.set_ylabel("Brake Torque (Nm)")
    ax5.set_title("Brake Torques - Front Axle")
    ax5.legend()
    ax5.grid(True)

    # 6) Brake Torques - Rear Wheels
    ax6 = plt.subplot(3, 4, 6)
    ax6.plot(t, df["tau_brake_rl_nm"], label="RL", alpha=0.8)
    ax6.plot(t, df["tau_brake_rr_nm"], label="RR", alpha=0.8)
    ax6.set_xlabel("t (s)")
    ax6.set_ylabel("Brake Torque (Nm)")
    ax6.set_title("Brake Torques - Rear Axle")
    ax6.legend()
    ax6.grid(True)

    # 7) Total Brake Torque (All Wheels)
    ax7 = plt.subplot(3, 4, 7)
    tau_brake_total = (df["tau_brake_fl_nm"] + df["tau_brake_fr_nm"] +
                       df["tau_brake_rl_nm"] + df["tau_brake_rr_nm"])
    ax7.plot(t, tau_brake_total, linewidth=2, color='red')
    ax7.set_xlabel("t (s)")
    ax7.set_ylabel("Total Brake Torque (Nm)")
    ax7.set_title("Total Brake Torque (All Wheels)")
    ax7.grid(True)

    # 8) Front/Rear Brake Balance
    ax8 = plt.subplot(3, 4, 8)
    tau_brake_front = df["tau_brake_fl_nm"] + df["tau_brake_fr_nm"]
    tau_brake_rear = df["tau_brake_rl_nm"] + df["tau_brake_rr_nm"]

    # Only compute ratio when braking is active
    brake_active = tau_brake_total > 10  # Threshold for active braking
    if brake_active.any():
        ratio = np.zeros(len(df))
        ratio[brake_active] = (tau_brake_rear[brake_active] /
                              (tau_brake_front[brake_active] + tau_brake_rear[brake_active]) * 100)
        ax8.plot(t, ratio, linewidth=2, color='purple')
        ax8.axhline(y=50, color='k', linestyle='--', linewidth=1, label='50/50 balance')
        ax8.set_xlabel("t (s)")
        ax8.set_ylabel("Rear Brake Torque (%)")
        ax8.set_title("Brake Balance\n(% rear torque)")
        ax8.set_ylim(0, 100)
        ax8.legend()
        ax8.grid(True)
    else:
        ax8.text(0.5, 0.5, "No braking detected", ha='center', va='center')

    # ========================================================================
    # Row 3: Wheel Speeds and Slip
    # ========================================================================

    # 9) Wheel Angular Velocities
    ax9 = plt.subplot(3, 4, 9)
    if "omega_rl_radps" in df.columns:
        wheel_speed_rl_rpm = df["omega_rl_radps"] * 60 / (2 * np.pi)
        wheel_speed_rr_rpm = df["omega_rr_radps"] * 60 / (2 * np.pi)
        ax9.plot(t, wheel_speed_rl_rpm, label="RL", linewidth=2)
        ax9.plot(t, wheel_speed_rr_rpm, label="RR", linewidth=2)
        ax9.set_xlabel("t (s)")
        ax9.set_ylabel("Wheel Speed (RPM)")
        ax9.set_title("Rear Wheel Angular Velocities")
        ax9.legend()
        ax9.grid(True)
    else:
        ax9.text(0.5, 0.5, "Wheel speed data unavailable", ha='center', va='center')

    # 10) Wheel Speed Asymmetry
    ax10 = plt.subplot(3, 4, 10)
    if "omega_rl_radps" in df.columns:
        omega_diff = df["omega_rr_radps"] - df["omega_rl_radps"]
        ax10.plot(t, omega_diff, linewidth=2, color='brown')
        add_zero_line(ax10)
        ax10.set_xlabel("t (s)")
        ax10.set_ylabel("Ω difference (rad/s)")
        ax10.set_title("Wheel Speed Asymmetry\n(RR - RL)")
        ax10.grid(True)
        symmetric_limits(ax10, omega_diff)

    # 11) Torque vs Wheel Speed (Operating Points)
    ax11 = plt.subplot(3, 4, 11)
    if "omega_rl_radps" in df.columns:
        wheel_speed_avg = (df["omega_rl_radps"] + df["omega_rr_radps"]) / 2
        scatter4 = ax11.scatter(wheel_speed_avg, tau_drive_total,
                               c=t, cmap='viridis', s=3, alpha=0.5)
        ax11.set_xlabel("Avg Wheel Speed (rad/s)")
        ax11.set_ylabel("Drive Torque (Nm)")
        ax11.set_title("Torque vs Speed\n(Operating Points)")
        ax11.grid(True)
        plt.colorbar(scatter4, ax=ax11, label='Time (s)')

    # 12) Power Delivery (Torque × Speed)
    ax12 = plt.subplot(3, 4, 12)
    if "omega_rl_radps" in df.columns:
        power_rl = df["tau_drive_rl_nm"] * df["omega_rl_radps"] / 1000  # kW
        power_rr = df["tau_drive_rr_nm"] * df["omega_rr_radps"] / 1000  # kW
        power_total = power_rl + power_rr

        ax12.plot(t, power_total, label="Total Wheel Power", linewidth=2, color='green')
        ax12.plot(t, power_rl, label="RL", alpha=0.5)
        ax12.plot(t, power_rr, label="RR", alpha=0.5)
        ax12.set_xlabel("t (s)")
        ax12.set_ylabel("Mechanical Power (kW)")
        ax12.set_title("Wheel Mechanical Power\n(τ × ω)")
        ax12.legend()
        ax12.grid(True)

    plt.tight_layout()
    return fig


def plot_combined_slip(df):
    """
    Figure 5: Combined Slip Analysis (sigma_x vs sigma_y, total slip)
    """
    # Check for slip data
    required = ["sigma_x_rl", "sigma_y_rl", "Fx_rl", "Fy_rl", "Fz_rl"]
    has_all, missing = check_columns(df, required)
    if not has_all:
        print(f"[WARN] Missing slip data: {missing}")
        return None

    fig = plt.figure(figsize=(20, 10))
    fig.suptitle("Figure 5: Combined Slip Analysis", fontsize=14, fontweight='bold')

    t = df["t_s"]

    # ========================================================================
    # Row 1: Slip Components
    # ========================================================================

    # 1) Longitudinal Slip (all wheels)
    ax1 = plt.subplot(2, 4, 1)
    ax1.plot(t, df["sigma_x_fl"], label="FL", alpha=0.7)
    ax1.plot(t, df["sigma_x_fr"], label="FR", alpha=0.7)
    ax1.plot(t, df["sigma_x_rl"], label="RL", linewidth=2)
    ax1.plot(t, df["sigma_x_rr"], label="RR", linewidth=2)
    add_zero_line(ax1)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("σx (dimensionless)")
    ax1.set_title("Longitudinal Slip Ratios")
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True)

    # 2) Lateral Slip (all wheels)
    ax2 = plt.subplot(2, 4, 2)
    ax2.plot(t, df["sigma_y_fl"], label="FL", alpha=0.7)
    ax2.plot(t, df["sigma_y_fr"], label="FR", alpha=0.7)
    ax2.plot(t, df["sigma_y_rl"], label="RL", linewidth=2)
    ax2.plot(t, df["sigma_y_rr"], label="RR", linewidth=2)
    add_zero_line(ax2)
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("σy (dimensionless)")
    ax2.set_title("Lateral Slip Ratios")
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True)

    # 3) Total Slip Magnitude - Rear Left
    ax3 = plt.subplot(2, 4, 3)
    sigma_total_rl = compute_total_slip(df["sigma_x_rl"], df["sigma_y_rl"])
    ax3.plot(t, sigma_total_rl, linewidth=2, color='purple')
    ax3.plot(t, np.abs(df["sigma_x_rl"]), alpha=0.5, label='|σx|')
    ax3.plot(t, np.abs(df["sigma_y_rl"]), alpha=0.5, label='|σy|')
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("Total Slip")
    ax3.set_title("Total Slip Magnitude - RL\n(√(σx² + σy²))")
    ax3.legend()
    ax3.grid(True)

    # 4) Slip Ratio (sigma_x vs sigma_y) - Rear Left
    ax4 = plt.subplot(2, 4, 4)
    scatter1 = ax4.scatter(df["sigma_x_rl"], df["sigma_y_rl"],
                          c=t, cmap='viridis', s=3, alpha=0.5)
    add_zero_line(ax4, axis='both')
    ax4.set_xlabel("σx (longitudinal)")
    ax4.set_ylabel("σy (lateral)")
    ax4.set_title("Combined Slip - Rear Left\n(σx vs σy)")
    ax4.axis('equal')
    ax4.grid(True)
    plt.colorbar(scatter1, ax=ax4, label='Time (s)')

    # ========================================================================
    # Row 2: Force Analysis
    # ========================================================================

    # 5) Force Vectors (Fx vs Fy) - Rear Left
    ax5 = plt.subplot(2, 4, 5)
    Fx_rl_kN = df["Fx_rl"] / 1000
    Fy_rl_kN = df["Fy_rl"] / 1000
    scatter2 = ax5.scatter(Fy_rl_kN, Fx_rl_kN, c=t, cmap='plasma', s=3, alpha=0.5)
    add_zero_line(ax5, axis='both')

    # Friction circle
    if "surface_mu" in df.columns and "Fz_rl" in df.columns:
        mu = df["surface_mu"].mean()
        Fz_avg = df["Fz_rl"].mean() / 1000
        F_max = mu * Fz_avg
        theta = np.linspace(0, 2*np.pi, 100)
        ax5.plot(F_max * np.cos(theta), F_max * np.sin(theta),
                'r--', linewidth=2, label=f'μFz limit')
        ax5.legend()

    ax5.set_xlabel("Fy (kN)")
    ax5.set_ylabel("Fx (kN)")
    ax5.set_title("Force Vector - Rear Left")
    ax5.axis('equal')
    ax5.grid(True)
    plt.colorbar(scatter2, ax=ax5, label='Time (s)')

    # 6) Slip vs Force Correlation (Fx vs sigma_x)
    ax6 = plt.subplot(2, 4, 6)
    scatter3 = ax6.scatter(df["sigma_x_rl"], Fx_rl_kN,
                          c=t, cmap='viridis', s=3, alpha=0.5)
    add_zero_line(ax6, axis='both')
    ax6.set_xlabel("σx")
    ax6.set_ylabel("Fx (kN)")
    ax6.set_title("Longitudinal Slip-Force\n(Fx vs σx)")
    ax6.grid(True)
    plt.colorbar(scatter3, ax=ax6, label='Time (s)')

    # 7) Slip vs Force Correlation (Fy vs sigma_y)
    ax7 = plt.subplot(2, 4, 7)
    scatter4 = ax7.scatter(df["sigma_y_rl"], Fy_rl_kN,
                          c=t, cmap='viridis', s=3, alpha=0.5)
    add_zero_line(ax7, axis='both')
    ax7.set_xlabel("σy")
    ax7.set_ylabel("Fy (kN)")
    ax7.set_title("Lateral Slip-Force\n(Fy vs σy)")
    ax7.grid(True)
    plt.colorbar(scatter4, ax=ax7, label='Time (s)')

    # 8) Friction Utilization vs Total Slip
    ax8 = plt.subplot(2, 4, 8)
    if "lambda_rl" in df.columns:
        scatter5 = ax8.scatter(sigma_total_rl, df["lambda_rl"],
                              c=t, cmap='plasma', s=3, alpha=0.5)
        ax8.axhline(y=1.0, color='r', linestyle='--', linewidth=2, label='saturation')
        ax8.set_xlabel("Total Slip σ")
        ax8.set_ylabel("Friction Utilization λ")
        ax8.set_title("Friction Utilization vs Total Slip\n(λ < 1 = saturated)")
        ax8.legend()
        ax8.grid(True)
        plt.colorbar(scatter5, ax=ax8, label='Time (s)')

    plt.tight_layout()
    return fig


def main():
    """Main function for standalone 3-DOF plotting."""
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "sim_out.csv"

    print(f"[INFO] Loading 3-DOF plots from: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"[INFO] Loaded {len(df)} rows, {len(df.columns)} columns")

    # Generate 3-DOF plots
    fig3 = plot_lateral_dynamics(df)
    fig4 = plot_wheel_torques(df)
    fig5 = plot_combined_slip(df)

    plt.show()


if __name__ == "__main__":
    main()
