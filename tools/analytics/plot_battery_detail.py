"""Figure 7 — Battery sensor detailed error analysis."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class BatteryDetailPlot(BasePlot):
    """3×3 battery truth-vs-measured, error bands, and histograms (Figure 7)."""

    _REQUIRED = [
        "batt_v_truth", "batt_v_meas",
        "batt_i_truth", "batt_i_meas",
        "batt_soc_truth", "batt_soc_meas",
    ]

    @property
    def _filename(self) -> str:
        return "fig07_battery_detail"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing battery columns: {missing}")
            return None

        v_err = df["batt_v_meas"] - df["batt_v_truth"]
        i_err = df["batt_i_meas"] - df["batt_i_truth"]
        soc_err = df["batt_soc_meas"] - df["batt_soc_truth"]
        v_rmse = float(np.sqrt(np.mean(v_err**2)))
        i_rmse = float(np.sqrt(np.mean(i_err**2)))
        soc_rmse = float(np.sqrt(np.mean(soc_err**2)))

        fig, axes = plt.subplots(3, 3, figsize=(18, 12))
        fig.suptitle("Battery Sensor Analysis: Truth vs Measured",
                     fontsize=16, fontweight="bold")

        def _truth_meas(ax, truth, meas, ylabel, title):
            ax.plot(df["t_s"], df[truth], label="Truth", color="blue")
            ax.plot(df["t_s"], df[meas], label="Measured",
                    color="red", linestyle="--", alpha=0.7)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.legend()
            ax.grid(True)

        def _error_band(ax, err, rmse, ylabel, title, label):
            ax.plot(df["t_s"], err, color="black")
            ax.axhline(y=0, color="red", linestyle="--", linewidth=0.5)
            ax.fill_between(df["t_s"], -rmse, rmse, alpha=0.2, color="blue",
                            label=f"±1σ = {rmse:.4f}{label}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.legend()
            ax.grid(True)

        _truth_meas(axes[0, 0], "batt_v_truth", "batt_v_meas",
                    "Voltage (V)", "Battery Voltage")
        _error_band(axes[0, 1], v_err, v_rmse,
                    "Error (V)", "Voltage Error (Measured - Truth)", "V")
        axes[0, 2].hist(v_err, bins=30, color="blue", alpha=0.7, edgecolor="black")
        axes[0, 2].axvline(x=0, color="red", linestyle="--", linewidth=2)
        axes[0, 2].set_xlabel("Error (V)")
        axes[0, 2].set_ylabel("Count")
        axes[0, 2].set_title("Voltage Error Distribution")
        axes[0, 2].grid(True)

        _truth_meas(axes[1, 0], "batt_i_truth", "batt_i_meas",
                    "Current (A)", "Battery Current")
        _error_band(axes[1, 1], i_err, i_rmse,
                    "Error (A)", "Current Error", "A")
        axes[1, 2].hist(i_err, bins=30, color="red", alpha=0.7, edgecolor="black")
        axes[1, 2].axvline(x=0, color="red", linestyle="--", linewidth=2)
        axes[1, 2].set_xlabel("Error (A)")
        axes[1, 2].set_ylabel("Count")
        axes[1, 2].set_title("Current Error Distribution")
        axes[1, 2].grid(True)

        _truth_meas(axes[2, 0], "batt_soc_truth", "batt_soc_meas",
                    "SOC (%)", "Battery State of Charge")
        _error_band(axes[2, 1], soc_err, soc_rmse,
                    "Error (%)", "SOC Error", "%")

        # Normalized errors
        v_err_norm = (v_err / v_rmse) * 100
        i_err_norm = (i_err / i_rmse) * 100
        soc_err_norm = (soc_err / soc_rmse) * 100
        axes[2, 2].plot(df["t_s"], v_err_norm, label="Voltage (%)", alpha=0.7)
        axes[2, 2].plot(df["t_s"], i_err_norm, label="Current (%)", alpha=0.7)
        axes[2, 2].plot(df["t_s"], soc_err_norm, label="SOC (%)", alpha=0.7)
        axes[2, 2].axhline(y=0, color="black", linestyle="--", linewidth=0.5)
        axes[2, 2].set_xlabel("Time (s)")
        axes[2, 2].set_ylabel("Relative Error (%)")
        axes[2, 2].set_title("Normalized Errors Over Time")
        axes[2, 2].legend()
        axes[2, 2].grid(True)

        plt.tight_layout()
        return fig
