"""Figure 1 — Vehicle dynamics, battery, and power."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class VehicleDynamicsPlot(BasePlot):
    """12-panel vehicle dynamics & battery plot (Figure 1)."""

    @property
    def _filename(self) -> str:
        return "fig01_vehicle_dynamics"

    def plot(self, df: pd.DataFrame):
        has_truth_meas, soc_col, v_col, i_col = DataUtils.get_battery_columns(df)

        fig = plt.figure(figsize=(20, 10))
        fig.suptitle("Figure 1: Vehicle Dynamics & Battery",
                     fontsize=14, fontweight="bold")

        # 1) Trajectory
        ax1 = plt.subplot(3, 4, 1)
        v = df["v_mps"].to_numpy()
        x = df["x_m"].to_numpy()
        y = df["y_m"].to_numpy()
        STOP_THRESH = 0.05

        if "gear_position" in df.columns:
            gear = df["gear_position"].to_numpy()
            direction = np.where(gear == 1, 1, np.where(gear == 2, -1, 0))
        else:
            direction = np.where(v > STOP_THRESH, 1,
                                 np.where(v < -STOP_THRESH, -1, 0))

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

        moving = np.abs(v) > STOP_THRESH
        stop_mask = np.zeros(len(v), dtype=bool)
        stop_mask[1:] = moving[:-1] & ~moving[1:]
        if stop_mask.any():
            ax1.scatter(x[stop_mask], y[stop_mask], s=60,
                        color="tab:orange", zorder=5, label="Stop", marker="o")
        ax1.scatter(x[0], y[0], s=60, color="tab:blue",
                    zorder=5, label="Start", marker="s")
        ax1.set_xlabel("x (m)")
        ax1.set_ylabel("y (m)")
        ax1.set_title("Trajectory (x vs y)")
        ax1.axis("equal")
        ax1.legend(fontsize=7)
        ax1.grid(True)

        # 2) Speed & Acceleration
        ax2 = plt.subplot(3, 4, 2)
        l_v, = ax2.plot(df["t_s"], df["v_mps"], label="velocity")
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("v (m/s)")
        ax2.set_title("Speed & Acceleration vs time")
        ax2.grid(True)
        if "a_long_mps2" in df.columns:
            ax2_twin = ax2.twinx()
            l_a, = ax2_twin.plot(df["t_s"], df["a_long_mps2"],
                                 "r-", alpha=0.5, label="accel")
            ax2_twin.set_ylabel("a (m/s²)", color="r")
            ax2_twin.tick_params(axis="y", labelcolor="r")
            ax2_twin.axhline(y=0, color="r", linestyle="--",
                             linewidth=0.5, alpha=0.6)
            a_min = float(df["a_long_mps2"].min())
            a_max = float(df["a_long_mps2"].max())
            a_abs = max(abs(a_min), abs(a_max))
            if a_abs > 1e-9:
                ax2_twin.set_ylim(-1.1 * a_abs, 1.1 * a_abs)
            ax2.legend(handles=[l_v, l_a], labels=["velocity", "accel"],
                       loc="best")

        # 3) Steering & Yaw
        ax3 = plt.subplot(3, 4, 3)
        handles, labels = [], []
        for col, lbl in [("steer_cmd_deg", "steer_cmd_deg"),
                         ("steer_deg", "steer_deg"),
                         ("delta_fl_deg", "fl_deg"),
                         ("delta_fr_deg", "fr_deg")]:
            if col in df.columns:
                kw = dict(linestyle="--", alpha=0.8) if "cmd" in col else {}
                if "delta" in col:
                    kw["alpha"] = 0.6
                l, = ax3.plot(df["t_s"], df[col], **kw)
                handles.append(l)
                labels.append(lbl)
        ax3.set_xlabel("t (s)")
        ax3.set_ylabel("Steering (deg)")
        ax3.set_title("Steering (cmd vs actual) & Yaw vs time")
        ax3.grid(True)
        ax3_yaw = ax3.twinx()
        if "yaw_deg" in df.columns:
            l_yaw, = ax3_yaw.plot(df["t_s"], df["yaw_deg"],
                                  color="orange", alpha=0.8)
            ax3_yaw.set_ylabel("Yaw (deg)")
            handles.append(l_yaw)
            labels.append("yaw_deg")
        if handles:
            ax3.legend(handles, labels, loc="best")

        # 4) Inputs
        ax4 = plt.subplot(3, 4, 4)
        l_m, = ax4.plot(df["t_s"], df["motor_nm"], label="motor_nm")
        ax4.set_xlabel("t (s)")
        ax4.set_ylabel("Motor torque (Nm)")
        ax4.set_title("Inputs vs time")
        ax4.grid(True)
        ax4_force = ax4.twinx()
        l_bf, = ax4_force.plot(df["t_s"], df["brake_force_kN"],
                               label="brake_force_kN", alpha=0.8, color="green")
        ax4_force.set_ylabel("Brake force (kN)")
        ax4_pct = ax4.twinx()
        ax4_pct.spines["right"].set_position(("axes", 1.12))
        l_bp, = ax4_pct.plot(df["t_s"], df["brake_pct"],
                              linestyle="--", alpha=0.7,
                              label="brake_pct", color="red")
        ax4_pct.set_ylabel("Brake pedal (%)")
        ax4_pct.set_ylim(0, 100)
        ax4.legend(handles=[l_m, l_bf, l_bp],
                   labels=["motor_nm", "brake_force_kN", "brake_pct"],
                   loc="best")

        # 5) SOC
        ax5 = plt.subplot(3, 4, 5)
        ax5.plot(df["t_s"], df[soc_col], label="SOC (%) - Truth", color="g")
        if has_truth_meas:
            ax5.plot(df["t_s"], df["batt_soc_meas"],
                     label="SOC (%) - Measured", color="g",
                     linestyle="--", alpha=0.7)
        ax5.set_xlabel("t (s)")
        ax5.set_ylabel("SOC (%)")
        ax5.set_title("Battery SOC vs time")
        ax5.legend()
        ax5.grid(True)

        # 6) Voltage
        ax6 = plt.subplot(3, 4, 6)
        ax6.plot(df["t_s"], df[v_col], label="Voltage (V) - Truth",
                 color="orange")
        if has_truth_meas:
            ax6.plot(df["t_s"], df["batt_v_meas"],
                     label="Voltage (V) - Measured", color="orange",
                     linestyle="--", alpha=0.7)
        ax6.set_xlabel("t (s)")
        ax6.set_ylabel("Voltage (V)")
        ax6.set_title("Battery Voltage vs time")
        ax6.legend()
        ax6.grid(True)

        # 7) Current
        ax7 = plt.subplot(3, 4, 7)
        ax7.plot(df["t_s"], df[i_col],
                 label="Battery Current (A) - Truth", color="purple")
        if has_truth_meas:
            ax7.plot(df["t_s"], df["batt_i_meas"],
                     label="Current (A) - Measured", color="purple",
                     linestyle="--", alpha=0.7)
        ax7.axhline(y=0, color="k", linestyle="--", linewidth=0.5)
        ax7.set_xlabel("t (s)")
        ax7.set_ylabel("Current (A)")
        ax7.set_title("Battery Current vs time\n(+discharge, -charge)")
        ax7.legend()
        ax7.grid(True)

        # 8) Net battery power
        ax8 = plt.subplot(3, 4, 8)
        net_power = df["motor_power_kW"] - df["regen_power_kW"]
        ax8.plot(df["t_s"], net_power, label="Net Battery Power (kW)", color="b")
        ax8.axhline(y=0, color="k", linestyle="--", linewidth=0.5)
        ax8.set_xlabel("t (s)")
        ax8.set_ylabel("Power (kW)")
        ax8.set_title("Net Battery Power vs time\n(+discharge, -charge)")
        ax8.legend()
        ax8.grid(True)

        # 9) Motor power
        ax9 = plt.subplot(3, 4, 9)
        ax9.plot(df["t_s"], df["motor_power_kW"],
                 label="Motor Power (kW)", color="r")
        ax9.set_xlabel("t (s)")
        ax9.set_ylabel("Power (kW)")
        ax9.set_title("Motor Power vs time")
        ax9.legend()
        ax9.grid(True)

        # 10) Regen power
        ax10 = plt.subplot(3, 4, 10)
        regen_active = df["regen_power_kW"] > 0.01
        if regen_active.any():
            ax10.fill_between(df["t_s"], 0, df["regen_power_kW"],
                              where=regen_active, alpha=0.3, color="green",
                              label="Regen Active")
        ax10.plot(df["t_s"], df["regen_power_kW"],
                  label="Regen Power (kW)", color="green")
        ax10.set_xlabel("t (s)")
        ax10.set_ylabel("Regenerative Power (kW)")
        ax10.set_title("Regenerative Braking Power vs Time")
        ax10.set_ylim(bottom=0)
        ax10.legend()
        ax10.grid(True)

        # 11) SOC vs Voltage
        ax11 = plt.subplot(3, 4, 11)
        ax11.scatter(df[soc_col], df[v_col], c=df["t_s"],
                     cmap="viridis", s=1, alpha=0.5)
        ax11.set_xlabel("SOC (%)")
        ax11.set_ylabel("Voltage (V)")
        ax11.set_title("Battery Characteristic Curve\n(SOC vs Voltage)")
        ax11.grid(True)
        cbar = plt.colorbar(ax11.collections[0], ax=ax11)
        cbar.set_label("Time (s)")

        # 12) Power vs Current
        ax12 = plt.subplot(3, 4, 12)
        motor_active = df["motor_power_kW"] > 0.01
        regen_active_mask = df["regen_power_kW"] > 0.01
        if motor_active.any():
            ax12.scatter(df.loc[motor_active, i_col],
                         df.loc[motor_active, "motor_power_kW"],
                         c="red", s=3, alpha=0.5, label="Motor (discharge)")
        if regen_active_mask.any():
            ax12.scatter(df.loc[regen_active_mask, i_col],
                         -df.loc[regen_active_mask, "regen_power_kW"],
                         c="green", s=3, alpha=0.5, label="Regen (charge)")
        ax12.axhline(y=0, color="k", linestyle="--", linewidth=0.5)
        ax12.axvline(x=0, color="k", linestyle="--", linewidth=0.5)
        ax12.set_xlabel("Battery Current (A)")
        ax12.set_ylabel("Power (kW)")
        ax12.set_title("Battery Operating Envelope\n(Power vs Current)")
        ax12.legend()
        ax12.grid(True)

        plt.tight_layout()
        return fig
