"""Figure 8 — Wheel speed sensor analysis (all 4 wheels)."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class WheelSensorsPlot(BasePlot):
    """4×2 wheel speed truth-vs-measured and error plots (Figure 8)."""

    _WHEELS = [
        ("fl", "Front Left"),
        ("fr", "Front Right"),
        ("rl", "Rear Left"),
        ("rr", "Rear Right"),
    ]

    @property
    def _filename(self) -> str:
        return "fig08_wheel_sensors"

    def plot(self, df: pd.DataFrame):
        required = [
            col
            for w, _ in self._WHEELS
            for col in (f"wheel_{w}_rps_truth", f"wheel_{w}_rps_meas")
        ]
        has_all, missing = DataUtils.check_columns(df, required)
        if not has_all:
            print(f"[WARN] Missing wheel sensor columns: {missing}")
            return None

        fig, axes = plt.subplots(4, 2, figsize=(16, 16))
        fig.suptitle("Wheel Speed Sensor Analysis: All 4 Wheels",
                     fontsize=16, fontweight="bold")

        for idx, (wheel, title) in enumerate(self._WHEELS):
            truth_col = f"wheel_{wheel}_rps_truth"
            meas_col = f"wheel_{wheel}_rps_meas"
            err = df[meas_col] - df[truth_col]
            rmse = float(np.sqrt(np.mean(err**2)))

            axes[idx, 0].plot(df["t_s"], df[truth_col],
                              label="Truth", color="blue")
            axes[idx, 0].plot(df["t_s"], df[meas_col], label="Measured",
                              color="red", linestyle="--", alpha=0.7)
            axes[idx, 0].set_xlabel("Time (s)")
            axes[idx, 0].set_ylabel("Speed (rps)")
            axes[idx, 0].set_title(f"{title} Wheel Speed")
            axes[idx, 0].legend()
            axes[idx, 0].grid(True)

            axes[idx, 1].plot(df["t_s"], err, color="black")
            axes[idx, 1].axhline(y=0, color="red", linestyle="--", linewidth=0.5)
            axes[idx, 1].fill_between(
                df["t_s"], -rmse, rmse, alpha=0.2, color="blue",
                label=f"±1σ = {rmse:.4f} rps")
            axes[idx, 1].set_xlabel("Time (s)")
            axes[idx, 1].set_ylabel("Error (rps)")
            axes[idx, 1].set_title(f"{title} Error")
            axes[idx, 1].legend()
            axes[idx, 1].grid(True)

        plt.tight_layout()
        return fig
