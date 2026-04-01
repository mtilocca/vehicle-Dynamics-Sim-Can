"""Figure 3 — 3-DOF lateral dynamics."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class LateralDynamicsPlot(BasePlot):
    """12-panel 3-DOF lateral dynamics plot (Figure 3)."""

    @property
    def _filename(self) -> str:
        return "fig03_lateral_dynamics"

    def plot(self, df: pd.DataFrame):
        required = ["t_s", "vy_mps", "yaw_rate_radps", "a_lat_mps2"]
        has_all, missing = DataUtils.check_columns(df, required)
        if not has_all:
            print(f"[WARN] Missing columns for lateral dynamics: {missing}")
            return None

        fig = plt.figure(figsize=(20, 12))
        fig.suptitle("Figure 3: 3-DOF Lateral Dynamics",
                     fontsize=14, fontweight="bold")
        t = df["t_s"]

        # 1) Lateral velocity
        ax1 = plt.subplot(3, 4, 1)
        ax1.plot(t, df["vy_mps"], linewidth=2, color="blue")
        DataUtils.add_zero_line(ax1)
        ax1.set_xlabel("t (s)")
        ax1.set_ylabel("Lateral Velocity (m/s)")
        ax1.set_title("Lateral Velocity (vy)\n(+left, -right)")
        ax1.grid(True)
        DataUtils.symmetric_limits(ax1, df["vy_mps"])

        # 2) Yaw rate
        ax2 = plt.subplot(3, 4, 2)
        yaw_rate_deg = DataUtils.rad_to_deg(df["yaw_rate_radps"])
        ax2.plot(t, yaw_rate_deg, linewidth=2, color="orange")
        DataUtils.add_zero_line(ax2)
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("Yaw Rate (deg/s)")
        ax2.set_title("Yaw Rate\n(+CCW, -CW)")
        ax2.grid(True)
        DataUtils.symmetric_limits(ax2, yaw_rate_deg)

        # 3) Lateral acceleration
        ax3 = plt.subplot(3, 4, 3)
        ax3.plot(t, df["a_lat_mps2"], linewidth=2, color="red")
        ax3.plot(t, df["a_lat_mps2"] / 9.81, linewidth=1,
                 alpha=0.5, label="g-force")
        DataUtils.add_zero_line(ax3)
        ax3.set_xlabel("t (s)")
        ax3.set_ylabel("Lateral Accel (m/s²)")
        ax3.set_title("Lateral Acceleration\n(+left, -right)")
        ax3.legend()
        ax3.grid(True)
        DataUtils.symmetric_limits(ax3, df["a_lat_mps2"])

        # 4) Sideslip angle
        ax4 = plt.subplot(3, 4, 4)
        beta_deg = DataUtils.compute_sideslip_angle(df)
        if beta_deg is not None:
            ax4.plot(t, beta_deg, linewidth=2, color="purple")
            DataUtils.add_zero_line(ax4)
            ax4.axhline(y=3, color="orange", linestyle=":", linewidth=1,
                        label="typical limit")
            ax4.axhline(y=-3, color="orange", linestyle=":", linewidth=1)
            ax4.set_xlabel("t (s)")
            ax4.set_ylabel("Sideslip Angle β (deg)")
            ax4.set_title("Vehicle Sideslip Angle\n(β = atan2(vy, vx))")
            ax4.legend()
            ax4.grid(True)
            DataUtils.symmetric_limits(ax4, beta_deg)
        else:
            ax4.text(0.5, 0.5, "Sideslip data unavailable",
                     ha="center", va="center")

        # 5) Front slip angles
        ax5 = plt.subplot(3, 4, 5)
        if "alpha_fl" in df.columns and "alpha_fr" in df.columns:
            ax5.plot(t, DataUtils.rad_to_deg(df["alpha_fl"]),
                     label="FL", alpha=0.8)
            ax5.plot(t, DataUtils.rad_to_deg(df["alpha_fr"]),
                     label="FR", alpha=0.8)
            DataUtils.add_zero_line(ax5)
            ax5.set_xlabel("t (s)")
            ax5.set_ylabel("Slip Angle (deg)")
            ax5.set_title("Front Slip Angles")
            ax5.legend()
            ax5.grid(True)
        else:
            ax5.text(0.5, 0.5, "Front slip angles unavailable",
                     ha="center", va="center")

        # 6) Rear slip angles
        ax6 = plt.subplot(3, 4, 6)
        if "alpha_rl" in df.columns and "alpha_rr" in df.columns:
            ax6.plot(t, DataUtils.rad_to_deg(df["alpha_rl"]),
                     label="RL", alpha=0.8)
            ax6.plot(t, DataUtils.rad_to_deg(df["alpha_rr"]),
                     label="RR", alpha=0.8)
            DataUtils.add_zero_line(ax6)
            ax6.set_xlabel("t (s)")
            ax6.set_ylabel("Slip Angle (deg)")
            ax6.set_title("Rear Slip Angles")
            ax6.legend()
            ax6.grid(True)
        else:
            ax6.text(0.5, 0.5, "Rear slip angles unavailable",
                     ha="center", va="center")

        # 7) All slip angles
        ax7 = plt.subplot(3, 4, 7)
        if all(c in df.columns
               for c in ["alpha_fl", "alpha_fr", "alpha_rl", "alpha_rr"]):
            ax7.plot(t, DataUtils.rad_to_deg(df["alpha_fl"]),
                     label="FL", alpha=0.6, linewidth=1)
            ax7.plot(t, DataUtils.rad_to_deg(df["alpha_fr"]),
                     label="FR", alpha=0.6, linewidth=1)
            ax7.plot(t, DataUtils.rad_to_deg(df["alpha_rl"]),
                     label="RL", alpha=0.8, linewidth=2)
            ax7.plot(t, DataUtils.rad_to_deg(df["alpha_rr"]),
                     label="RR", alpha=0.8, linewidth=2)
            DataUtils.add_zero_line(ax7)
            ax7.set_xlabel("t (s)")
            ax7.set_ylabel("Slip Angle (deg)")
            ax7.set_title("All Tire Slip Angles")
            ax7.legend(loc="upper right", fontsize=8)
            ax7.grid(True)
        else:
            ax7.text(0.5, 0.5, "Slip angle data unavailable",
                     ha="center", va="center")

        # 8) Phase portrait
        ax8 = plt.subplot(3, 4, 8)
        scatter = ax8.scatter(yaw_rate_deg, df["vy_mps"],
                              c=t, cmap="viridis", s=3, alpha=0.6)
        DataUtils.add_zero_line(ax8, axis="both")
        ax8.set_xlabel("Yaw Rate (deg/s)")
        ax8.set_ylabel("Lateral Velocity (m/s)")
        ax8.set_title("Lateral Phase Portrait\n(vy vs yaw_rate)")
        ax8.grid(True)
        plt.colorbar(scatter, ax=ax8, label="Time (s)")

        # 9) Steering vs Yaw rate
        ax9 = plt.subplot(3, 4, 9)
        if "steer_deg" in df.columns:
            ax9_steer = ax9.twinx()
            l1, = ax9.plot(t, yaw_rate_deg, label="Yaw Rate",
                           color="blue", linewidth=2)
            l2, = ax9_steer.plot(t, df["steer_deg"], label="Steering",
                                  color="green", alpha=0.6, linestyle="--")
            DataUtils.add_zero_line(ax9)
            ax9.set_xlabel("t (s)")
            ax9.set_ylabel("Yaw Rate (deg/s)", color="blue")
            ax9_steer.set_ylabel("Steering (deg)", color="green")
            ax9.set_title("Yaw Rate Response to Steering")
            ax9.legend(handles=[l1, l2], loc="upper right")
            ax9.grid(True)
        else:
            ax9.text(0.5, 0.5, "Steering data unavailable",
                     ha="center", va="center")

        # 10) Understeer diagram
        ax10 = plt.subplot(3, 4, 10)
        if "steer_deg" in df.columns:
            mask = np.abs(df["steer_deg"]) > 0.5
            if mask.any():
                scatter2 = ax10.scatter(
                    df.loc[mask, "steer_deg"],
                    df.loc[mask, "a_lat_mps2"],
                    c=df.loc[mask, "v_mps"], cmap="plasma", s=5, alpha=0.5)
                DataUtils.add_zero_line(ax10)
                ax10.set_xlabel("Steering Angle (deg)")
                ax10.set_ylabel("Lateral Accel (m/s²)")
                ax10.set_title("Understeer Analysis\n(a_lat vs δ)")
                ax10.grid(True)
                plt.colorbar(scatter2, ax=ax10, label="Speed (m/s)")
            else:
                ax10.text(0.5, 0.5, "No steering input",
                          ha="center", va="center")
        else:
            ax10.text(0.5, 0.5, "Steering data unavailable",
                      ha="center", va="center")

        # 11) Yaw rate vs Lateral accel
        ax11 = plt.subplot(3, 4, 11)
        scatter3 = ax11.scatter(df["a_lat_mps2"], yaw_rate_deg,
                                c=t, cmap="viridis", s=3, alpha=0.6)
        DataUtils.add_zero_line(ax11, axis="both")
        ax11.set_xlabel("Lateral Accel (m/s²)")
        ax11.set_ylabel("Yaw Rate (deg/s)")
        ax11.set_title("Lateral Accel vs Yaw Rate")
        ax11.grid(True)
        plt.colorbar(scatter3, ax=ax11, label="Time (s)")

        # 12) Trajectory with velocity vectors
        ax12 = plt.subplot(3, 4, 12)
        step = max(1, len(df) // 50)
        idx = range(0, len(df), step)
        ax12.plot(df["x_m"], df["y_m"], "b-", linewidth=1,
                  alpha=0.5, label="path")
        for i in idx:
            x = df["x_m"].iloc[i]
            y = df["y_m"].iloc[i]
            yaw = DataUtils.deg_to_rad(df["yaw_deg"].iloc[i])
            vx_b = df["v_mps"].iloc[i]
            vy_b = df["vy_mps"].iloc[i]
            vx_g = vx_b * np.cos(yaw) - vy_b * np.sin(yaw)
            vy_g = vx_b * np.sin(yaw) + vy_b * np.cos(yaw)
            ax12.arrow(x, y, vx_g * 0.5, vy_g * 0.5,
                       head_width=0.3, head_length=0.2,
                       fc="red", ec="red", alpha=0.6)
        ax12.set_xlabel("x (m)")
        ax12.set_ylabel("y (m)")
        ax12.set_title("Trajectory with Velocity Vectors\n"
                       "(red = velocity direction)")
        ax12.axis("equal")
        ax12.grid(True)
        ax12.legend()

        plt.tight_layout()
        return fig
