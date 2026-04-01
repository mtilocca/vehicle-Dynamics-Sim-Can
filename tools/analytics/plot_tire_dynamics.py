"""Figure 2 — Dugoff tire dynamics."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class TireDynamicsPlot(BasePlot):
    """12-panel Dugoff tire forces, slip, and friction-circle plot (Figure 2)."""

    _REQUIRED = [
        "Fx_fl", "Fx_fr", "Fx_rl", "Fx_rr",
        "Fy_fl", "Fy_fr", "Fy_rl", "Fy_rr",
        "Fz_fl", "Fz_fr", "Fz_rl", "Fz_rr",
        "sigma_x_fl", "sigma_x_fr", "sigma_x_rl", "sigma_x_rr",
        "sigma_y_fl", "sigma_y_fr", "sigma_y_rl", "sigma_y_rr",
        "lambda_fl", "lambda_fr", "lambda_rl", "lambda_rr",
        "surface_mu",
    ]

    @property
    def _filename(self) -> str:
        return "fig02_tire_dynamics"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[INFO] Tire columns not found: {missing[:5]} ...")
            return None

        fig = plt.figure(figsize=(20, 12))
        fig.suptitle("Figure 2: Tire Dynamics (Dugoff Model)",
                     fontsize=14, fontweight="bold")
        t = df["t_s"]

        # 1) Longitudinal forces
        ax1 = plt.subplot(3, 4, 1)
        ax1.plot(t, df["Fx_fl"] / 1000, label="FL", alpha=0.8)
        ax1.plot(t, df["Fx_fr"] / 1000, label="FR", alpha=0.8)
        ax1.plot(t, df["Fx_rl"] / 1000, label="RL", linewidth=2)
        ax1.plot(t, df["Fx_rr"] / 1000, label="RR", linewidth=2)
        DataUtils.add_zero_line(ax1)
        ax1.set_xlabel("t (s)")
        ax1.set_ylabel("Fx (kN)")
        ax1.set_title("Longitudinal Tire Forces\n(+drive, -brake)")
        ax1.legend(loc="upper right", fontsize=8)
        ax1.grid(True)

        # 2) Total longitudinal
        ax2 = plt.subplot(3, 4, 2)
        Fx_total = df["Fx_fl"] + df["Fx_fr"] + df["Fx_rl"] + df["Fx_rr"]
        Fx_rear = df["Fx_rl"] + df["Fx_rr"]
        ax2.plot(t, Fx_total / 1000, label="Fx_total", color="blue", linewidth=2)
        ax2.plot(t, Fx_rear / 1000, label="Fx_rear (driven)", color="red", alpha=0.7)
        DataUtils.add_zero_line(ax2)
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("Fx (kN)")
        ax2.set_title("Total Longitudinal Force")
        ax2.legend()
        ax2.grid(True)

        # 3) Lateral forces
        ax3 = plt.subplot(3, 4, 3)
        for col, lbl in [("Fy_fl", "FL"), ("Fy_fr", "FR"),
                          ("Fy_rl", "RL"), ("Fy_rr", "RR")]:
            ax3.plot(t, df[col] / 1000, label=lbl, alpha=0.8)
        DataUtils.add_zero_line(ax3)
        ax3.set_xlabel("t (s)")
        ax3.set_ylabel("Fy (kN)")
        ax3.set_title("Lateral Tire Forces\n(+left, -right)")
        ax3.legend(loc="upper right", fontsize=8)
        ax3.grid(True)

        # 4) Normal loads
        ax4 = plt.subplot(3, 4, 4)
        for col, lbl in [("Fz_fl", "FL"), ("Fz_fr", "FR"),
                          ("Fz_rl", "RL"), ("Fz_rr", "RR")]:
            ax4.plot(t, df[col] / 1000, label=lbl)
        ax4.set_xlabel("t (s)")
        ax4.set_ylabel("Fz (kN)")
        ax4.set_title("Normal Loads (Weight Distribution)")
        ax4.legend(loc="upper right", fontsize=8)
        ax4.grid(True)

        # 5) Longitudinal slip
        ax5 = plt.subplot(3, 4, 5)
        ax5.plot(t, df["sigma_x_fl"], label="FL", alpha=0.8)
        ax5.plot(t, df["sigma_x_fr"], label="FR", alpha=0.8)
        ax5.plot(t, df["sigma_x_rl"], label="RL", linewidth=2)
        ax5.plot(t, df["sigma_x_rr"], label="RR", linewidth=2)
        DataUtils.add_zero_line(ax5)
        ax5.axhline(y=0.1, color="r", linestyle=":", linewidth=0.5,
                    label="peak slip")
        ax5.axhline(y=-0.1, color="r", linestyle=":", linewidth=0.5)
        ax5.set_xlabel("t (s)")
        ax5.set_ylabel("σx (dimensionless)")
        ax5.set_title("Longitudinal Slip Ratio\n(+accel, -brake)")
        ax5.legend(loc="upper right", fontsize=8)
        ax5.grid(True)

        # 6) Lateral slip
        ax6 = plt.subplot(3, 4, 6)
        for col, lbl in [("sigma_y_fl", "FL"), ("sigma_y_fr", "FR"),
                          ("sigma_y_rl", "RL"), ("sigma_y_rr", "RR")]:
            ax6.plot(t, df[col], label=lbl, alpha=0.8)
        DataUtils.add_zero_line(ax6)
        ax6.set_xlabel("t (s)")
        ax6.set_ylabel("σy (dimensionless)")
        ax6.set_title("Lateral Slip Ratio\n(≈ slip angle)")
        ax6.legend(loc="upper right", fontsize=8)
        ax6.grid(True)

        # 7) Friction utilization
        ax7 = plt.subplot(3, 4, 7)
        for col, lbl in [("lambda_fl", "FL"), ("lambda_fr", "FR"),
                          ("lambda_rl", "RL"), ("lambda_rr", "RR")]:
            ax7.plot(t, df[col], label=lbl, alpha=0.8)
        ax7.axhline(y=1.0, color="r", linestyle="--", linewidth=1,
                    label="saturation")
        ax7.set_xlabel("t (s)")
        ax7.set_ylabel("λ (dimensionless)")
        ax7.set_title("Friction Utilization\n(λ<1 = saturated)")
        ax7.legend(loc="upper right", fontsize=8)
        ax7.grid(True)
        ax7.set_ylim(0, max(2.0, df[["lambda_fl", "lambda_fr",
                                      "lambda_rl", "lambda_rr"]].max().max() * 1.1))

        # 8) Surface friction
        ax8 = plt.subplot(3, 4, 8)
        ax8.plot(t, df["surface_mu"], label="μ", color="brown", linewidth=2)
        ax8.axhline(y=0.85, color="g", linestyle=":", label="dry pavement")
        ax8.axhline(y=0.72, color="orange", linestyle=":", label="compact gravel")
        ax8.axhline(y=0.30, color="r", linestyle=":", label="wet dust")
        ax8.set_xlabel("t (s)")
        ax8.set_ylabel("μ (dimensionless)")
        ax8.set_title("Surface Friction Coefficient")
        ax8.legend(loc="upper right", fontsize=8)
        ax8.grid(True)
        ax8.set_ylim(0, 1.0)

        # 9) Friction circle — rear
        ax9 = plt.subplot(3, 4, 9)
        Fx_r = (df["Fx_rl"] + df["Fx_rr"]) / 1000
        Fy_r = (df["Fy_rl"] + df["Fy_rr"]) / 1000
        Fz_r = (df["Fz_rl"] + df["Fz_rr"]) / 1000
        mu = df["surface_mu"].mean()
        theta = np.linspace(0, 2 * np.pi, 100)
        F_max = mu * Fz_r.mean()
        ax9.plot(F_max * np.cos(theta), F_max * np.sin(theta),
                 "r--", label=f"μ={mu:.2f} limit")
        scatter = ax9.scatter(Fy_r, Fx_r, c=t, cmap="viridis", s=5, alpha=0.5)
        ax9.set_xlabel("Fy_rear (kN)")
        ax9.set_ylabel("Fx_rear (kN)")
        ax9.set_title("Friction Circle - Rear Axle")
        ax9.axis("equal")
        ax9.legend(loc="upper right", fontsize=8)
        ax9.grid(True)
        plt.colorbar(scatter, ax=ax9, label="Time (s)")

        # 10) Friction circle — front
        ax10 = plt.subplot(3, 4, 10)
        Fx_f = (df["Fx_fl"] + df["Fx_fr"]) / 1000
        Fy_f = (df["Fy_fl"] + df["Fy_fr"]) / 1000
        Fz_f = (df["Fz_fl"] + df["Fz_fr"]) / 1000
        F_max_f = mu * Fz_f.mean()
        ax10.plot(F_max_f * np.cos(theta), F_max_f * np.sin(theta),
                  "r--", label=f"μ={mu:.2f} limit")
        scatter2 = ax10.scatter(Fy_f, Fx_f, c=t, cmap="viridis", s=5, alpha=0.5)
        ax10.set_xlabel("Fy_front (kN)")
        ax10.set_ylabel("Fx_front (kN)")
        ax10.set_title("Friction Circle - Front Axle")
        ax10.axis("equal")
        ax10.legend(loc="upper right", fontsize=8)
        ax10.grid(True)
        plt.colorbar(scatter2, ax=ax10, label="Time (s)")

        # 11) Load transfer
        ax11 = plt.subplot(3, 4, 11)
        if "a_long_mps2" in df.columns:
            Fz_front = df["Fz_fl"] + df["Fz_fr"]
            Fz_rear_ax = df["Fz_rl"] + df["Fz_rr"]
            load_ratio = Fz_rear_ax / (Fz_front + Fz_rear_ax) * 100
            ax11.scatter(df["a_long_mps2"], load_ratio,
                         c=t, cmap="viridis", s=5, alpha=0.5)
            ax11.axhline(y=50, color="k", linestyle="--",
                         linewidth=0.5, label="static")
            ax11.set_xlabel("Longitudinal Accel (m/s²)")
            ax11.set_ylabel("Rear Axle Load (%)")
            ax11.set_title("Load Transfer\n(accel→rear, brake→front)")
            ax11.legend()
            ax11.grid(True)
        else:
            ax11.text(0.5, 0.5, "a_long_mps2 not in CSV",
                      ha="center", va="center")
            ax11.set_title("Load Transfer (no data)")

        # 12) Slip vs Force
        ax12 = plt.subplot(3, 4, 12)
        sigma_x = df["sigma_x_rl"]
        Fx_rl_kN = df["Fx_rl"] / 1000
        scatter3 = ax12.scatter(sigma_x, Fx_rl_kN,
                                c=t, cmap="viridis", s=5, alpha=0.5,
                                label="actual")
        Fz_avg = df["Fz_rl"].mean()
        mu_avg = df["surface_mu"].mean()
        ax12.axhline(y=mu_avg * Fz_avg / 1000, color="r",
                     linestyle="--", label="μFz limit")
        ax12.axhline(y=-mu_avg * Fz_avg / 1000, color="r", linestyle="--")
        ax12.set_xlabel("σx (slip ratio)")
        ax12.set_ylabel("Fx (kN)")
        ax12.set_title("Slip-Force Characteristic\n(Rear-Left Wheel)")
        ax12.legend(loc="upper right", fontsize=8)
        ax12.grid(True)
        ax12.set_xlim(-0.35, 0.35)
        plt.colorbar(scatter3, ax=ax12, label="Time (s)")

        plt.tight_layout()
        return fig
