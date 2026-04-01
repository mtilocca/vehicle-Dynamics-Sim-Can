"""Figure 6 — Battery & wheel sensor overview."""
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class SensorOverviewPlot(BasePlot):
    """2×2 overview of battery and wheel sensors (Figure 6)."""

    _REQUIRED = [
        "batt_soc_truth", "batt_soc_meas",
        "batt_v_truth", "batt_v_meas",
        "batt_i_truth", "batt_i_meas",
        "wheel_fl_rps_truth", "wheel_fl_rps_meas",
    ]

    @property
    def _filename(self) -> str:
        return "fig06_sensor_overview"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing sensor columns: {missing}")
            return None

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Battery & Wheel Speed Sensors",
                     fontsize=16, fontweight="bold")

        for ax, truth, meas, ylabel, title in [
            (axes[0, 0], "batt_soc_truth", "batt_soc_meas",
             "SOC (%)", "Battery State of Charge"),
            (axes[0, 1], "batt_v_truth", "batt_v_meas",
             "Voltage (V)", "Battery Voltage"),
            (axes[1, 0], "batt_i_truth", "batt_i_meas",
             "Current (A)", "Battery Current"),
            (axes[1, 1], "wheel_fl_rps_truth", "wheel_fl_rps_meas",
             "Wheel Speed (rps)", "Front-Left Wheel Speed"),
        ]:
            ax.plot(df["t_s"], df[truth], "b-", label="Truth", linewidth=2)
            ax.plot(df["t_s"], df[meas], "r--", label="Measured", alpha=0.7)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.legend()
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        return fig
