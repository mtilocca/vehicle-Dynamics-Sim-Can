#!/usr/bin/env python3
"""
sim_plotter.py - Vehicle Dynamics Simulation Plotter

Usage:
    python3 sim_plotter.py [sim_out.csv] [--save output.png]

Plots 6 panels (2×3):
    1. Trajectory (x vs y)
    2. Speed & longitudinal acceleration vs time
    3. Steering angles & yaw vs time
    4. Motor torque cmd & brake force vs time
    5. Wheel speeds (all 4) vs time
    6. Lateral velocity & yaw rate vs time
"""

import sys
import argparse
from typing import Optional
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    return df


def plot(df: pd.DataFrame, save_path: Optional[str] = None) -> None:
    t = df["t_s"]

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Vehicle Dynamics Simulation — 3-DOF Plant", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    # ------------------------------------------------------------------
    # Panel 1 — Trajectory
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df["x_m"], df["y_m"], "b-", linewidth=1.5)
    ax1.scatter([df["x_m"].iloc[0]], [df["y_m"].iloc[0]], color="green", s=60, zorder=5, label="Start")
    ax1.scatter([df["x_m"].iloc[-1]], [df["y_m"].iloc[-1]], color="red", s=60, zorder=5, label="End")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("Trajectory")
    ax1.legend(fontsize=8, loc="best")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 2 — Speed & longitudinal acceleration
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, df["v_mps"] * 3.6, "b-", linewidth=1.5, label="Speed [km/h]")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("Speed [km/h]", color="b")
    ax2.tick_params(axis="y", labelcolor="b")
    ax2.set_title("Speed & Longitudinal Accel")
    ax2.grid(True, alpha=0.3)

    ax2r = ax2.twinx()
    ax2r.plot(t, df["a_long_mps2"], "r--", linewidth=1.2, label="a_long [m/s²]")
    ax2r.set_ylabel("a_long [m/s²]", color="r")
    ax2r.tick_params(axis="y", labelcolor="r")

    lines2  = ax2.get_lines() + ax2r.get_lines()
    labels2 = [l.get_label() for l in lines2]
    ax2.legend(lines2, labels2, fontsize=8, loc='best')

    # ------------------------------------------------------------------
    # Panel 3 — Steering angles & yaw
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(t, df["steer_deg"], "b-",  linewidth=1.5, label="Virtual steer [°]")
    ax3.plot(t, df["delta_fl_deg"], "c--", linewidth=1.0, label="FL [°]")
    ax3.plot(t, df["delta_fr_deg"], "m--", linewidth=1.0, label="FR [°]")
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("Steer angle [°]", color="b")
    ax3.tick_params(axis="y", labelcolor="b")
    ax3.set_title("Steering & Yaw")
    ax3.grid(True, alpha=0.3)

    ax3r = ax3.twinx()
    ax3r.plot(t, df["yaw_deg"], "k-", linewidth=1.2, label="Yaw [°]")
    ax3r.set_ylabel("Yaw [°]", color="k")
    ax3r.tick_params(axis="y", labelcolor="k")

    lines3  = ax3.get_lines() + ax3r.get_lines()
    labels3 = [l.get_label() for l in lines3]
    ax3.legend(lines3, labels3, fontsize=8, loc="best")

    # ------------------------------------------------------------------
    # Panel 4 — Motor torque & brake force
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(t, df["motor_nm_cmd"] / 1000.0, "b-", linewidth=1.5, label="Torque cmd [kNm]")
    ax4.plot(t, df["motor_torque_nm"] / 1000.0, "b--", linewidth=1.0, label="Torque actual [kNm]")
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("Torque [kNm]", color="b")
    ax4.tick_params(axis="y", labelcolor="b")
    ax4.set_title("Motor Torque & Brake Force")
    ax4.grid(True, alpha=0.3)

    ax4r = ax4.twinx()
    ax4r.plot(t, df["brake_force_kN"], "r-", linewidth=1.5, label="Brake force [kN]")
    ax4r.set_ylabel("Brake force [kN]", color="r")
    ax4r.tick_params(axis="y", labelcolor="r")

    lines4  = ax4.get_lines() + ax4r.get_lines()
    labels4 = [l.get_label() for l in lines4]
    ax4.legend(lines4, labels4, fontsize=8, loc="best")

    # ------------------------------------------------------------------
    # Panel 5 — Wheel speeds
    # ------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(t, df["wheel_fl_rps"] * 60.0 / (2 * np.pi), "b-",  lw=1.2, label="FL")
    ax5.plot(t, df["wheel_fr_rps"] * 60.0 / (2 * np.pi), "c--", lw=1.2, label="FR")
    ax5.plot(t, df["wheel_rl_rps"] * 60.0 / (2 * np.pi), "r-",  lw=1.2, label="RL")
    ax5.plot(t, df["wheel_rr_rps"] * 60.0 / (2 * np.pi), "m--", lw=1.2, label="RR")
    ax5.set_xlabel("time [s]")
    ax5.set_ylabel("Wheel speed [RPM]")
    ax5.set_title("Wheel Speeds")
    ax5.legend(fontsize=8, loc="best")
    ax5.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 6 — Lateral velocity & yaw rate
    # ------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(t, df["vy_mps"], "b-", linewidth=1.5, label="vy [m/s]")
    ax6.set_xlabel("time [s]")
    ax6.set_ylabel("Lateral velocity [m/s]", color="b")
    ax6.tick_params(axis="y", labelcolor="b")
    ax6.set_title("Lateral Velocity & Yaw Rate")
    ax6.grid(True, alpha=0.3)

    ax6r = ax6.twinx()
    ax6r.plot(t, np.degrees(df["yaw_rate_radps"]), "r-", linewidth=1.2, label="Yaw rate [°/s]")
    ax6r.set_ylabel("Yaw rate [°/s]", color="r")
    ax6r.tick_params(axis="y", labelcolor="r")

    lines6  = ax6.get_lines() + ax6r.get_lines()
    labels6 = [l.get_label() for l in lines6]
    ax6.legend(lines6, labels6, fontsize=8, loc="best")

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved to {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot vehicle dynamics simulation results")
    parser.add_argument("csv", nargs="?", default="sim_out.csv", help="CSV log file (default: sim_out.csv)")
    parser.add_argument("--save", metavar="FILE", help="Save figure to file instead of displaying")
    args = parser.parse_args()

    try:
        df = load_csv(args.csv)
    except FileNotFoundError:
        print(f"Error: CSV file not found: {args.csv}", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {len(df)} rows from {args.csv}")
    print(f"Duration: {df['t_s'].iloc[-1]:.1f} s   Max speed: {df['v_mps'].max() * 3.6:.1f} km/h")

    plot(df, save_path=args.save)


if __name__ == "__main__":
    main()
