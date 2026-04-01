"""Figure 13 — Radar sensor validation."""
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class RadarValidationPlot(BasePlot):
    """2×2 radar range, Doppler, angle, and status plot (Figure 13)."""

    _REQUIRED = [
        "radar_target_range_m", "radar_target_rel_vel_mps",
        "radar_target_angle_deg", "radar_status", "v_mps",
    ]

    @property
    def _filename(self) -> str:
        return "fig13_radar_validation"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing radar columns: {missing}")
            return None

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Radar Sensor - Under Development",
                     fontsize=16, fontweight="bold")

        axes[0, 0].plot(df["t_s"], df["radar_target_range_m"],
                        "b-", linewidth=2)
        axes[0, 0].axhline(y=50.0, color="k", linestyle="--", alpha=0.3)
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel("Range (m)")
        axes[0, 0].set_title("Radar Range to Target")
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].plot(df["t_s"], df["radar_target_rel_vel_mps"],
                        "r-", linewidth=2)
        axes[0, 1].plot(df["t_s"], df["v_mps"], "k--", alpha=0.3,
                        label="Expected")
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("Range Rate (m/s)")
        axes[0, 1].set_title("Radar Doppler")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].plot(df["t_s"], df["radar_target_angle_deg"],
                        "g-", linewidth=2)
        axes[1, 0].axhline(y=0, color="k", linestyle="--", alpha=0.3)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Angle (deg)")
        axes[1, 0].set_title("Radar Angle")
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].plot(df["t_s"], df["radar_status"], "k-", linewidth=2)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Status")
        axes[1, 1].set_title("Radar Status")
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        return fig
