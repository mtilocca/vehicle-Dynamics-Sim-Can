#!/usr/bin/env python3
"""
sim_plotter.py - Vehicle Dynamics Simulation Plotter

Usage:
    python3 sim_plotter.py [sim_out.csv] [--save PREFIX]

Figure 1 — 7 panels (2×4, last slot empty):
    1. Trajectory (x vs y)
    2. Speed [km/h]
    3. Longitudinal acceleration breakdown
    4. Steering angles & yaw
    5. Motor torque & brake force
    6. Wheel speeds
    7. Lateral velocity & yaw rate

Figure 2 — 6 panels (2×3):
    1. Sideslip angle β [°]
    2. Lateral acceleration vs centripetal term
    3. G-G diagram (a_lat vs a_long scatter)
    4. Actuator commands (torque % & brake %)
    5. Body-frame velocity components (vx, vy)
    6. Sim loop timing [µs]
"""

import sys
import csv
import argparse
from typing import Optional, Dict
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


def load_csv(path: str) -> Dict[str, np.ndarray]:
    """Load CSV into a dict of numpy arrays (pandas-free)."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError("Empty CSV file")
    keys = [k.strip() for k in rows[0].keys()]
    data: Dict[str, np.ndarray] = {}
    for k in keys:
        data[k] = np.array([float(r[k]) for r in rows])
    return data


def plot_figure1(df: Dict[str, np.ndarray], save_path: Optional[str] = None) -> None:
    t = df["t_s"]
    coupling  = df["vy_mps"] * df["yaw_rate_radps"]
    a_net     = df["a_long_mps2"] - coupling

    fig = plt.figure(figsize=(20, 10))
    fig.suptitle("Vehicle Dynamics Simulation — 3-DOF Plant", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.45, wspace=0.38)

    # ------------------------------------------------------------------
    # Panel 1 — Trajectory
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df["x_m"], df["y_m"], "b-", linewidth=1.5)
    ax1.scatter([df["x_m"][0]],  [df["y_m"][0]],  color="green", s=60, zorder=5, label="Start")
    ax1.scatter([df["x_m"][-1]], [df["y_m"][-1]], color="red",   s=60, zorder=5, label="End")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("Trajectory")
    ax1.legend(fontsize=8, loc="best")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 2 — Speed only (clean)
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, df["v_mps"] * 3.6, "b-", linewidth=1.8, label="Speed")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("Speed [km/h]")
    ax2.set_title("Speed")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8, loc="best")

    # ------------------------------------------------------------------
    # Panel 3 — Longitudinal acceleration breakdown
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(t, df["a_long_mps2"], "r-",  linewidth=1.5, label="a_long total [m/s²]")
    ax3.plot(t, coupling,           "m--", linewidth=1.2, label="vy·ψ̇ coupling [m/s²]")
    ax3.plot(t, a_net,              "g:",  linewidth=1.5, label="a_net (total − coupling)")
    ax3.axhline(0, color="k", linewidth=0.6, linestyle=":")
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("Acceleration [m/s²]")
    ax3.set_title("Longitudinal Accel Breakdown")
    ax3.legend(fontsize=7, loc="best")
    ax3.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 4 — Steering angles & yaw
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[0, 3])
    ax4.plot(t, df["steer_deg"],    "b-",  linewidth=1.5, label="Virtual steer [°]")
    ax4.plot(t, df["delta_fl_deg"], "c--", linewidth=1.0, label="FL [°]")
    ax4.plot(t, df["delta_fr_deg"], "m--", linewidth=1.0, label="FR [°]")
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("Steer angle [°]", color="b")
    ax4.tick_params(axis="y", labelcolor="b")
    ax4.set_title("Steering & Yaw")
    ax4.grid(True, alpha=0.3)

    ax4r = ax4.twinx()
    ax4r.plot(t, df["yaw_deg"], "k-", linewidth=1.2, label="Yaw [°]")
    ax4r.set_ylabel("Yaw [°]", color="k")
    ax4r.tick_params(axis="y", labelcolor="k")

    lines4  = ax4.get_lines() + ax4r.get_lines()
    labels4 = [l.get_label() for l in lines4]
    ax4.legend(lines4, labels4, fontsize=7, loc="lower left")

    # ------------------------------------------------------------------
    # Panel 5 — Motor torque & brake force
    # ------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 0])
    ax5.plot(t, df["motor_nm_cmd"]    / 1000.0, "b-",  linewidth=1.5, label="Torque cmd [kNm]")
    ax5.plot(t, df["motor_torque_nm"] / 1000.0, "b--", linewidth=1.0, label="Torque actual [kNm]")
    ax5.set_xlabel("time [s]")
    ax5.set_ylabel("Torque [kNm]", color="b")
    ax5.tick_params(axis="y", labelcolor="b")
    ax5.set_title("Motor Torque & Brake Force")
    ax5.grid(True, alpha=0.3)

    ax5r = ax5.twinx()
    ax5r.plot(t, df["brake_force_kN"], "r-", linewidth=1.5, label="Brake force [kN]")
    ax5r.set_ylabel("Brake force [kN]", color="r")
    ax5r.tick_params(axis="y", labelcolor="r")

    lines5  = ax5.get_lines() + ax5r.get_lines()
    labels5 = [l.get_label() for l in lines5]
    ax5.legend(lines5, labels5, fontsize=7, loc="best")

    # ------------------------------------------------------------------
    # Panel 6 — Wheel speeds
    # ------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 1])
    ax6.plot(t, df["wheel_fl_rps"] * 60.0 / (2 * np.pi), "b-",  lw=1.2, label="FL")
    ax6.plot(t, df["wheel_fr_rps"] * 60.0 / (2 * np.pi), "c--", lw=1.2, label="FR")
    ax6.plot(t, df["wheel_rl_rps"] * 60.0 / (2 * np.pi), "r-",  lw=1.2, label="RL")
    ax6.plot(t, df["wheel_rr_rps"] * 60.0 / (2 * np.pi), "m--", lw=1.2, label="RR")
    ax6.set_xlabel("time [s]")
    ax6.set_ylabel("Wheel speed [RPM]")
    ax6.set_title("Wheel Speeds")
    ax6.legend(fontsize=8, loc="best")
    ax6.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 7 — Lateral velocity & yaw rate
    # ------------------------------------------------------------------
    ax7 = fig.add_subplot(gs[1, 2])
    ax7.plot(t, df["vy_mps"], "b-", linewidth=1.5, label="vy [m/s]")
    ax7.set_xlabel("time [s]")
    ax7.set_ylabel("Lateral velocity [m/s]", color="b")
    ax7.tick_params(axis="y", labelcolor="b")
    ax7.set_title("Lateral Velocity & Yaw Rate")
    ax7.grid(True, alpha=0.3)

    ax7r = ax7.twinx()
    ax7r.plot(t, np.degrees(df["yaw_rate_radps"]), "r-", linewidth=1.2, label="Yaw rate [°/s]")
    ax7r.set_ylabel("Yaw rate [°/s]", color="r")
    ax7r.tick_params(axis="y", labelcolor="r")

    lines7  = ax7.get_lines() + ax7r.get_lines()
    labels7 = [l.get_label() for l in lines7]
    ax7.legend(lines7, labels7, fontsize=8, loc="best")

    # ------------------------------------------------------------------
    # Panel 8 — Lateral force estimate & friction limit
    # Fy_est = m · a_lat  (total lateral force on the vehicle body [kN])
    # Friction limit = μ · m · g  (peak available lateral force [kN])
    # Utilisation = |Fy_est| / limit × 100 %
    # ------------------------------------------------------------------
    MASS_KG = 218000.0
    MU      = 0.72
    G       = 9.81
    Fy_kN   = MASS_KG * df["a_lat_mps2"] / 1000.0
    Fy_lim  = MU * MASS_KG * G / 1000.0           # constant limit [kN]
    util_pct = np.abs(Fy_kN) / Fy_lim * 100.0

    ax8 = fig.add_subplot(gs[1, 3])
    ax8.plot(t, Fy_kN,            "b-",  linewidth=1.5, label="Fy est [kN]")
    ax8.axhline( Fy_lim, color="r", linewidth=1.0, linestyle="--", label=f"+limit {Fy_lim:.0f} kN")
    ax8.axhline(-Fy_lim, color="r", linewidth=1.0, linestyle="--")
    ax8.axhline(0,        color="k", linewidth=0.5, linestyle=":")
    ax8.set_xlabel("time [s]")
    ax8.set_ylabel("Lateral force [kN]", color="b")
    ax8.tick_params(axis="y", labelcolor="b")
    ax8.set_title("Lateral Force & Friction Limit")
    ax8.grid(True, alpha=0.3)

    ax8r = ax8.twinx()
    ax8r.plot(t, util_pct, "m-", linewidth=1.0, alpha=0.7, label="Utilisation [%]")
    ax8r.set_ylabel("Utilisation [%]", color="m")
    ax8r.tick_params(axis="y", labelcolor="m")
    ax8r.set_ylim(0, 120)

    lines8  = ax8.get_lines()[:2] + ax8r.get_lines()
    labels8 = [l.get_label() for l in lines8]
    ax8.legend(lines8, labels8, fontsize=7, loc="upper right")

    if save_path:
        out = save_path + "_fig1.png"
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Figure 1 saved to {out}")
    else:
        plt.show()


def plot_figure2(df: Dict[str, np.ndarray], save_path: Optional[str] = None) -> None:
    t = df["t_s"]

    # Derived quantities
    vx_safe      = np.where(df["v_mps"] > 0.1, df["v_mps"], 0.1)
    sideslip_deg = np.degrees(np.arctan2(df["vy_mps"], vx_safe))
    centripetal  = df["v_mps"] * df["yaw_rate_radps"]          # vx·ψ̇ [m/s²]
    tq_max = np.abs(df["motor_nm_cmd"]).max()
    torque_pct = df["motor_nm_cmd"] / tq_max * 100.0 if tq_max > 0 else np.zeros_like(df["motor_nm_cmd"])

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Vehicle Dynamics Simulation — Extended Analysis", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.48, wspace=0.38)

    # ------------------------------------------------------------------
    # Panel 1 — Sideslip angle β [°]
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, sideslip_deg, "b-", linewidth=1.5)
    ax1.axhline(0, color="k", linewidth=0.6, linestyle=":")
    ax1.fill_between(t, sideslip_deg, 0, alpha=0.15, color="blue")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("Sideslip β [°]")
    ax1.set_title("Body Sideslip Angle β")
    ax1.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 2 — Lateral acceleration vs centripetal term
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, df["a_lat_mps2"], "b-",  linewidth=1.5, label="a_lat total [m/s²]")
    ax2.plot(t, centripetal,      "r--", linewidth=1.2, label="vx·ψ̇ centripetal [m/s²]")
    ax2.plot(t, df["a_lat_mps2"] - centripetal, "g:", linewidth=1.2, label="Fy/m (tire forces only)")
    ax2.axhline(0, color="k", linewidth=0.6, linestyle=":")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("Acceleration [m/s²]")
    ax2.set_title("Lateral Accel & Centripetal Term")
    ax2.legend(fontsize=7, loc="best")
    ax2.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 3 — G-G diagram (friction circle usage)
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    sc = ax3.scatter(df["a_long_mps2"], df["a_lat_mps2"],
                     c=t, cmap="viridis", s=4, alpha=0.7)
    plt.colorbar(sc, ax=ax3, label="time [s]", pad=0.01)
    # friction circle reference (μ·g = 0.72 × 9.81)
    mu_g = 0.72 * 9.81
    theta = np.linspace(0, 2 * np.pi, 200)
    ax3.plot(mu_g * np.cos(theta), mu_g * np.sin(theta),
             "r--", linewidth=1.0, alpha=0.6, label=f"μ·g = {mu_g:.1f} m/s²")
    ax3.axhline(0, color="k", linewidth=0.5)
    ax3.axvline(0, color="k", linewidth=0.5)
    ax3.set_xlabel("a_long [m/s²]")
    ax3.set_ylabel("a_lat [m/s²]")
    ax3.set_title("G-G Diagram (Friction Circle Usage)")
    ax3.set_aspect("equal", adjustable="datalim")
    ax3.legend(fontsize=7, loc="upper right")
    ax3.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 4 — Actuator commands (torque % & brake %)
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(t, torque_pct,            "b-",  linewidth=1.5, label="Drive torque cmd [%]")
    ax4.plot(t, df["brake_pct_cmd"],   "r-",  linewidth=1.5, label="Brake cmd [%]")
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("Command [%]")
    ax4.set_title("Actuator Commands")
    ax4.set_ylim(-10, 110)
    ax4.legend(fontsize=8, loc="best")
    ax4.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 5 — Body-frame velocity components (vx, vy)
    # ------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(t, df["v_mps"],   "b-",  linewidth=1.5, label="vx [m/s]")
    ax5.plot(t, df["vy_mps"],  "r-",  linewidth=1.5, label="vy [m/s]")
    speed_total = np.hypot(df["v_mps"], df["vy_mps"])
    ax5.plot(t, speed_total,   "k--", linewidth=1.0, label="|v| total [m/s]")
    ax5.axhline(0, color="k", linewidth=0.5, linestyle=":")
    ax5.set_xlabel("time [s]")
    ax5.set_ylabel("Velocity [m/s]")
    ax5.set_title("Body-Frame Velocity Components")
    ax5.legend(fontsize=8, loc="best")
    ax5.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 6 — Sim loop timing [µs]
    # ------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2])
    loop_us = np.clip(df["loop_time_us"], 0, None)   # drop negative sentinel values
    pos = loop_us[loop_us > 0]
    mean_us = pos.mean() if len(pos) else 0.0
    ax6.plot(t, loop_us, "g-", linewidth=0.8, alpha=0.7, label="Loop time [µs]")
    ax6.axhline(mean_us, color="r", linewidth=1.2,
                linestyle="--", label=f"Mean {mean_us:.0f} µs")
    ax6.set_xlabel("time [s]")
    ax6.set_ylabel("Loop time [µs]")
    ax6.set_title("Simulation Step Timing")
    ax6.legend(fontsize=8, loc="best")
    ax6.grid(True, alpha=0.3)

    if save_path:
        out = save_path + "_fig2.png"
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Figure 2 saved to {out}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot vehicle dynamics simulation results")
    parser.add_argument("csv",    nargs="?", default="sim_out.csv",
                        help="CSV log file (default: sim_out.csv)")
    parser.add_argument("--save", metavar="PREFIX",
                        help="Save figures to PREFIX_fig1.png and PREFIX_fig2.png")
    args = parser.parse_args()

    try:
        df = load_csv(args.csv)
    except FileNotFoundError:
        print(f"Error: CSV file not found: {args.csv}", file=sys.stderr)
        sys.exit(1)

    n = len(df["t_s"])
    print(f"Loaded {n} rows from {args.csv}")
    print(f"Duration: {df['t_s'][-1]:.1f} s   Max speed: {df['v_mps'].max() * 3.6:.1f} km/h")

    plot_figure1(df, save_path=args.save)
    plot_figure2(df, save_path=args.save)

    if not args.save:
        plt.show()


if __name__ == "__main__":
    main()
