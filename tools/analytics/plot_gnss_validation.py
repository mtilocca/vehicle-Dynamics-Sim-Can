"""Figure 12 — GNSS sensor validation."""
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class GnssValidationPlot(BasePlot):
    """2×2 GNSS trajectory, altitude, velocity, and fix-quality plot (Figure 12)."""

    _REQUIRED = [
        "gnss_lat_deg", "gnss_lon_deg", "gnss_alt_m",
        "gnss_vn_mps", "gnss_ve_mps",
        "gnss_fix_type", "gnss_sat_count",
    ]

    @property
    def _filename(self) -> str:
        return "fig12_gnss_validation"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing GNSS columns: {missing}")
            return None

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("GNSS Sensor (GPS) - Under Development",
                     fontsize=16, fontweight="bold")

        axes[0, 0].plot(df["gnss_lon_deg"], df["gnss_lat_deg"],
                        "r-", linewidth=2)
        axes[0, 0].set_xlabel("Longitude (deg)")
        axes[0, 0].set_ylabel("Latitude (deg)")
        axes[0, 0].set_title("GNSS Trajectory (WGS84)")
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].axis("equal")

        axes[0, 1].plot(df["t_s"], df["gnss_alt_m"], "g-", linewidth=2)
        axes[0, 1].axhline(y=0, color="k", linestyle="--", alpha=0.3)
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("Altitude (m MSL)")
        axes[0, 1].set_title("GNSS Altitude")
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].plot(df["t_s"], df["gnss_vn_mps"],
                        "b-", label="North", linewidth=2)
        axes[1, 0].plot(df["t_s"], df["gnss_ve_mps"],
                        "r-", label="East", linewidth=2)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Velocity (m/s)")
        axes[1, 0].set_title("GNSS Velocity (NED Frame)")
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].plot(df["t_s"], df["gnss_fix_type"],
                        "g-", linewidth=2, label="Fix Type")
        axes[1, 1].plot(df["t_s"], df["gnss_sat_count"],
                        "b--", linewidth=2, label="Sat Count")
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Value")
        axes[1, 1].set_title("GNSS Quality")
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        return fig
