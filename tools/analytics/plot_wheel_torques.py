"""Figure 4 — Wheel torque distribution and drive dynamics."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class WheelTorquesPlot(BasePlot):
    """12-panel wheel torque distribution plot (Figure 4)."""

    _REQUIRED = [
        "tau_drive_rl_nm", "tau_drive_rr_nm",
        "tau_brake_fl_nm", "tau_brake_fr_nm",
        "tau_brake_rl_nm", "tau_brake_rr_nm",
    ]

    @property
    def _filename(self) -> str:
        return "fig04_wheel_torques"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing torque columns: {missing}")
            return None

        fig = plt.figure(figsize=(20, 10))
        fig.suptitle("Figure 4: Wheel Torque Distribution",
                     fontsize=14, fontweight="bold")
        t = df["t_s"]

        # 1) Drive torques — rear
        ax1 = plt.subplot(3, 4, 1)
        ax1.plot(t, df["tau_drive_rl_nm"], label="RL", linewidth=2)
        ax1.plot(t, df["tau_drive_rr_nm"], label="RR", linewidth=2)
        ax1.set_xlabel("t (s)")
        ax1.set_ylabel("Torque (Nm)")
        ax1.set_title("Drive Torques - Rear Axle")
        ax1.legend()
        ax1.grid(True)

        # 2) Total drive torque
        ax2 = plt.subplot(3, 4, 2)
        tau_drive_total = df["tau_drive_rl_nm"] + df["tau_drive_rr_nm"]
        ax2.plot(t, tau_drive_total, linewidth=2, color="green")
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("Total Torque (Nm)")
        ax2.set_title("Total Drive Torque (RL + RR)")
        ax2.grid(True)

        # 3) Drive torque asymmetry
        ax3 = plt.subplot(3, 4, 3)
        tau_drive_diff = df["tau_drive_rr_nm"] - df["tau_drive_rl_nm"]
        ax3.plot(t, tau_drive_diff, linewidth=2, color="orange")
        DataUtils.add_zero_line(ax3)
        ax3.set_xlabel("t (s)")
        ax3.set_ylabel("Torque Difference (Nm)")
        ax3.set_title("Drive Torque Asymmetry\n(RR - RL)")
        ax3.grid(True)
        DataUtils.symmetric_limits(ax3, tau_drive_diff)

        # 4) Motor cmd vs total drive
        ax4 = plt.subplot(3, 4, 4)
        if "motor_nm" in df.columns:
            ax4.plot(t, df["motor_nm"], label="Motor Command",
                     linestyle="--", alpha=0.7)
            ax4.plot(t, tau_drive_total, label="Total Drive Torque", linewidth=2)
            ax4.set_xlabel("t (s)")
            ax4.set_ylabel("Torque (Nm)")
            ax4.set_title("Motor Cmd vs Actual Drive Torque")
            ax4.legend()
            ax4.grid(True)

        # 5) Brake torques — front
        ax5 = plt.subplot(3, 4, 5)
        ax5.plot(t, df["tau_brake_fl_nm"], label="FL", alpha=0.8)
        ax5.plot(t, df["tau_brake_fr_nm"], label="FR", alpha=0.8)
        ax5.set_xlabel("t (s)")
        ax5.set_ylabel("Brake Torque (Nm)")
        ax5.set_title("Brake Torques - Front Axle")
        ax5.legend()
        ax5.grid(True)

        # 6) Brake torques — rear
        ax6 = plt.subplot(3, 4, 6)
        ax6.plot(t, df["tau_brake_rl_nm"], label="RL", alpha=0.8)
        ax6.plot(t, df["tau_brake_rr_nm"], label="RR", alpha=0.8)
        ax6.set_xlabel("t (s)")
        ax6.set_ylabel("Brake Torque (Nm)")
        ax6.set_title("Brake Torques - Rear Axle")
        ax6.legend()
        ax6.grid(True)

        # 7) Total brake torque
        ax7 = plt.subplot(3, 4, 7)
        tau_brake_total = (df["tau_brake_fl_nm"] + df["tau_brake_fr_nm"] +
                           df["tau_brake_rl_nm"] + df["tau_brake_rr_nm"])
        ax7.plot(t, tau_brake_total, linewidth=2, color="red")
        ax7.set_xlabel("t (s)")
        ax7.set_ylabel("Total Brake Torque (Nm)")
        ax7.set_title("Total Brake Torque (All Wheels)")
        ax7.grid(True)

        # 8) Front/rear brake balance
        ax8 = plt.subplot(3, 4, 8)
        brake_active = tau_brake_total > 10
        if brake_active.any():
            tau_brake_front = df["tau_brake_fl_nm"] + df["tau_brake_fr_nm"]
            tau_brake_rear = df["tau_brake_rl_nm"] + df["tau_brake_rr_nm"]
            ratio = np.zeros(len(df))
            ratio[brake_active] = (
                tau_brake_rear[brake_active] /
                (tau_brake_front[brake_active] + tau_brake_rear[brake_active]) * 100
            )
            ax8.plot(t, ratio, linewidth=2, color="purple")
            ax8.axhline(y=50, color="k", linestyle="--",
                        linewidth=1, label="50/50 balance")
            ax8.set_xlabel("t (s)")
            ax8.set_ylabel("Rear Brake Torque (%)")
            ax8.set_title("Brake Balance\n(% rear torque)")
            ax8.set_ylim(0, 100)
            ax8.legend()
            ax8.grid(True)
        else:
            ax8.text(0.5, 0.5, "No braking detected",
                     ha="center", va="center")

        # 9) Wheel angular velocities
        ax9 = plt.subplot(3, 4, 9)
        if "omega_rl_radps" in df.columns:
            rpm_rl = df["omega_rl_radps"] * 60 / (2 * np.pi)
            rpm_rr = df["omega_rr_radps"] * 60 / (2 * np.pi)
            ax9.plot(t, rpm_rl, label="RL", linewidth=2)
            ax9.plot(t, rpm_rr, label="RR", linewidth=2)
            ax9.set_xlabel("t (s)")
            ax9.set_ylabel("Wheel Speed (RPM)")
            ax9.set_title("Rear Wheel Angular Velocities")
            ax9.legend()
            ax9.grid(True)
        else:
            ax9.text(0.5, 0.5, "Wheel speed data unavailable",
                     ha="center", va="center")

        # 10) Wheel speed asymmetry
        ax10 = plt.subplot(3, 4, 10)
        if "omega_rl_radps" in df.columns:
            omega_diff = df["omega_rr_radps"] - df["omega_rl_radps"]
            ax10.plot(t, omega_diff, linewidth=2, color="brown")
            DataUtils.add_zero_line(ax10)
            ax10.set_xlabel("t (s)")
            ax10.set_ylabel("Ω difference (rad/s)")
            ax10.set_title("Wheel Speed Asymmetry\n(RR - RL)")
            ax10.grid(True)
            DataUtils.symmetric_limits(ax10, omega_diff)

        # 11) Torque vs speed operating points
        ax11 = plt.subplot(3, 4, 11)
        if "omega_rl_radps" in df.columns:
            wheel_speed_avg = (df["omega_rl_radps"] + df["omega_rr_radps"]) / 2
            scatter4 = ax11.scatter(wheel_speed_avg, tau_drive_total,
                                    c=t, cmap="viridis", s=3, alpha=0.5)
            ax11.set_xlabel("Avg Wheel Speed (rad/s)")
            ax11.set_ylabel("Drive Torque (Nm)")
            ax11.set_title("Torque vs Speed\n(Operating Points)")
            ax11.grid(True)
            plt.colorbar(scatter4, ax=ax11, label="Time (s)")

        # 12) Wheel mechanical power
        ax12 = plt.subplot(3, 4, 12)
        if "omega_rl_radps" in df.columns:
            power_rl = df["tau_drive_rl_nm"] * df["omega_rl_radps"] / 1000
            power_rr = df["tau_drive_rr_nm"] * df["omega_rr_radps"] / 1000
            ax12.plot(t, power_rl + power_rr,
                      label="Total Wheel Power", linewidth=2, color="green")
            ax12.plot(t, power_rl, label="RL", alpha=0.5)
            ax12.plot(t, power_rr, label="RR", alpha=0.5)
            ax12.set_xlabel("t (s)")
            ax12.set_ylabel("Mechanical Power (kW)")
            ax12.set_title("Wheel Mechanical Power\n(τ × ω)")
            ax12.legend()
            ax12.grid(True)

        plt.tight_layout()
        return fig
