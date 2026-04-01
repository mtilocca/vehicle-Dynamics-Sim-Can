"""Figure 5 — Combined slip analysis."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class CombinedSlipPlot(BasePlot):
    """8-panel combined slip analysis plot (Figure 5)."""

    _REQUIRED = ["sigma_x_rl", "sigma_y_rl", "Fx_rl", "Fy_rl", "Fz_rl"]

    @property
    def _filename(self) -> str:
        return "fig05_combined_slip"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing slip data: {missing}")
            return None

        fig = plt.figure(figsize=(20, 10))
        fig.suptitle("Figure 5: Combined Slip Analysis",
                     fontsize=14, fontweight="bold")
        t = df["t_s"]

        # 1) Longitudinal slip — all wheels
        ax1 = plt.subplot(2, 4, 1)
        for col, lbl, lw in [("sigma_x_fl", "FL", 1), ("sigma_x_fr", "FR", 1),
                               ("sigma_x_rl", "RL", 2), ("sigma_x_rr", "RR", 2)]:
            ax1.plot(t, df[col], label=lbl, alpha=0.7, linewidth=lw)
        DataUtils.add_zero_line(ax1)
        ax1.set_xlabel("t (s)")
        ax1.set_ylabel("σx (dimensionless)")
        ax1.set_title("Longitudinal Slip Ratios")
        ax1.legend(loc="upper right", fontsize=8)
        ax1.grid(True)

        # 2) Lateral slip — all wheels
        ax2 = plt.subplot(2, 4, 2)
        for col, lbl, lw in [("sigma_y_fl", "FL", 1), ("sigma_y_fr", "FR", 1),
                               ("sigma_y_rl", "RL", 2), ("sigma_y_rr", "RR", 2)]:
            ax2.plot(t, df[col], label=lbl, alpha=0.7, linewidth=lw)
        DataUtils.add_zero_line(ax2)
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("σy (dimensionless)")
        ax2.set_title("Lateral Slip Ratios")
        ax2.legend(loc="upper right", fontsize=8)
        ax2.grid(True)

        # 3) Total slip magnitude — RL
        ax3 = plt.subplot(2, 4, 3)
        sigma_total_rl = DataUtils.compute_total_slip(
            df["sigma_x_rl"], df["sigma_y_rl"])
        ax3.plot(t, sigma_total_rl, linewidth=2, color="purple")
        ax3.plot(t, np.abs(df["sigma_x_rl"]), alpha=0.5, label="|σx|")
        ax3.plot(t, np.abs(df["sigma_y_rl"]), alpha=0.5, label="|σy|")
        ax3.set_xlabel("t (s)")
        ax3.set_ylabel("Total Slip")
        ax3.set_title("Total Slip Magnitude - RL\n(√(σx² + σy²))")
        ax3.legend()
        ax3.grid(True)

        # 4) Combined slip scatter — RL
        ax4 = plt.subplot(2, 4, 4)
        scatter1 = ax4.scatter(df["sigma_x_rl"], df["sigma_y_rl"],
                               c=t, cmap="viridis", s=3, alpha=0.5)
        DataUtils.add_zero_line(ax4, axis="both")
        ax4.set_xlabel("σx (longitudinal)")
        ax4.set_ylabel("σy (lateral)")
        ax4.set_title("Combined Slip - Rear Left\n(σx vs σy)")
        ax4.axis("equal")
        ax4.grid(True)
        plt.colorbar(scatter1, ax=ax4, label="Time (s)")

        # 5) Force vector with friction circle — RL
        ax5 = plt.subplot(2, 4, 5)
        Fx_rl_kN = df["Fx_rl"] / 1000
        Fy_rl_kN = df["Fy_rl"] / 1000
        scatter2 = ax5.scatter(Fy_rl_kN, Fx_rl_kN,
                               c=t, cmap="plasma", s=3, alpha=0.5)
        DataUtils.add_zero_line(ax5, axis="both")
        if "surface_mu" in df.columns:
            mu = df["surface_mu"].mean()
            Fz_avg = df["Fz_rl"].mean() / 1000
            F_max = mu * Fz_avg
            theta = np.linspace(0, 2 * np.pi, 100)
            ax5.plot(F_max * np.cos(theta), F_max * np.sin(theta),
                     "r--", linewidth=2, label="μFz limit")
            ax5.legend()
        ax5.set_xlabel("Fy (kN)")
        ax5.set_ylabel("Fx (kN)")
        ax5.set_title("Force Vector - Rear Left")
        ax5.axis("equal")
        ax5.grid(True)
        plt.colorbar(scatter2, ax=ax5, label="Time (s)")

        # 6) Fx vs σx
        ax6 = plt.subplot(2, 4, 6)
        scatter3 = ax6.scatter(df["sigma_x_rl"], Fx_rl_kN,
                               c=t, cmap="viridis", s=3, alpha=0.5)
        DataUtils.add_zero_line(ax6, axis="both")
        ax6.set_xlabel("σx")
        ax6.set_ylabel("Fx (kN)")
        ax6.set_title("Longitudinal Slip-Force\n(Fx vs σx)")
        ax6.grid(True)
        plt.colorbar(scatter3, ax=ax6, label="Time (s)")

        # 7) Fy vs σy
        ax7 = plt.subplot(2, 4, 7)
        scatter4 = ax7.scatter(df["sigma_y_rl"], Fy_rl_kN,
                               c=t, cmap="viridis", s=3, alpha=0.5)
        DataUtils.add_zero_line(ax7, axis="both")
        ax7.set_xlabel("σy")
        ax7.set_ylabel("Fy (kN)")
        ax7.set_title("Lateral Slip-Force\n(Fy vs σy)")
        ax7.grid(True)
        plt.colorbar(scatter4, ax=ax7, label="Time (s)")

        # 8) Friction utilization vs total slip
        ax8 = plt.subplot(2, 4, 8)
        if "lambda_rl" in df.columns:
            scatter5 = ax8.scatter(sigma_total_rl, df["lambda_rl"],
                                   c=t, cmap="plasma", s=3, alpha=0.5)
            ax8.axhline(y=1.0, color="r", linestyle="--",
                        linewidth=2, label="saturation")
            ax8.set_xlabel("Total Slip σ")
            ax8.set_ylabel("Friction Utilization λ")
            ax8.set_title("Friction Utilization vs Total Slip\n(λ < 1 = saturated)")
            ax8.legend()
            ax8.grid(True)
            plt.colorbar(scatter5, ax=ax8, label="Time (s)")

        plt.tight_layout()
        return fig
