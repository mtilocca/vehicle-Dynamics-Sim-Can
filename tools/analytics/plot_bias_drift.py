"""Figure 10 — Sensor bias drift (low-pass filtered error analysis)."""
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from .base_plot import BasePlot
from .data_utils import DataUtils


class BiasDriftPlot(BasePlot):
    """2×2 bias-drift plot using low-pass filtered sensor errors (Figure 10)."""

    _REQUIRED = [
        "batt_v_truth", "batt_v_meas",
        "batt_i_truth", "batt_i_meas",
        "batt_soc_truth", "batt_soc_meas",
    ]

    @property
    def _filename(self) -> str:
        return "fig10_bias_drift"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing columns for bias drift: {missing}")
            return None

        dt = float(df["t_s"].iloc[1] - df["t_s"].iloc[0])
        fs = 1.0 / dt

        v_err = df["batt_v_meas"] - df["batt_v_truth"]
        i_err = df["batt_i_meas"] - df["batt_i_truth"]
        soc_err = df["batt_soc_meas"] - df["batt_soc_truth"]

        v_rmse = float(np.sqrt(np.mean(v_err**2)))
        i_rmse = float(np.sqrt(np.mean(i_err**2)))
        soc_rmse = float(np.sqrt(np.mean(soc_err**2)))

        v_bias = DataUtils.lowpass_filter(v_err.values, cutoff=0.05, fs=fs)
        i_bias = DataUtils.lowpass_filter(i_err.values, cutoff=0.05, fs=fs)
        soc_bias = DataUtils.lowpass_filter(soc_err.values, cutoff=0.05, fs=fs)

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Sensor Bias Drift Analysis (Low-Pass Filtered)",
                     fontsize=16, fontweight="bold")

        for ax, err, bias, ylabel, title, color in [
            (axes[0, 0], v_err,   v_bias,   "Voltage Error (V)",  "Voltage Sensor Bias Drift",   "blue"),
            (axes[0, 1], i_err,   i_bias,   "Current Error (A)",  "Current Sensor Bias Drift",   "red"),
            (axes[1, 0], soc_err, soc_bias, "SOC Error (%)",      "SOC Estimation Bias Drift",   "green"),
        ]:
            ax.plot(df["t_s"], err, color="lightgray",
                    alpha=0.5, label="Instantaneous Error")
            ax.plot(df["t_s"], bias, color=color,
                    linewidth=2, label="Bias (Slow Drift)")
            ax.axhline(y=0, color="black", linestyle="--", linewidth=0.5)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.legend()
            ax.grid(True)

        axes[1, 1].plot(df["t_s"], (v_bias / v_rmse) * 100,
                        label="Voltage (%)", color="blue")
        axes[1, 1].plot(df["t_s"], (i_bias / i_rmse) * 100,
                        label="Current (%)", color="orange")
        axes[1, 1].plot(df["t_s"], (soc_bias / soc_rmse) * 100,
                        label="SOC (%)", color="green")
        axes[1, 1].axhline(y=0, color="black", linestyle="--", linewidth=0.5)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Normalized Bias (%)")
        axes[1, 1].set_title("All Sensor Bias Drift (Normalized)")
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        plt.tight_layout()
        return fig
