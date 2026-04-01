"""Figure 9 — Sensor noise frequency spectrum (FFT/PSD)."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.fft import fft, fftfreq

from .base_plot import BasePlot
from .data_utils import DataUtils


class NoiseSpectrumPlot(BasePlot):
    """2×2 FFT power-spectral-density plot for battery & wheel errors (Figure 9)."""

    _REQUIRED = [
        "batt_v_truth", "batt_v_meas",
        "batt_i_truth", "batt_i_meas",
        "batt_soc_truth", "batt_soc_meas",
        "wheel_fl_rps_truth", "wheel_fl_rps_meas",
        "wheel_fr_rps_truth", "wheel_fr_rps_meas",
        "wheel_rl_rps_truth", "wheel_rl_rps_meas",
        "wheel_rr_rps_truth", "wheel_rr_rps_meas",
    ]

    @property
    def _filename(self) -> str:
        return "fig09_noise_spectrum"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing columns for noise spectrum: {missing}")
            return None

        dt = float(df["t_s"].iloc[1] - df["t_s"].iloc[0])
        N = len(df)
        xf = fftfreq(N, dt)[:N // 2]

        v_err = (df["batt_v_meas"] - df["batt_v_truth"]).values
        i_err = (df["batt_i_meas"] - df["batt_i_truth"]).values
        soc_err = (df["batt_soc_meas"] - df["batt_soc_truth"]).values
        wheel_err = (
            (df["wheel_fl_rps_meas"] - df["wheel_fl_rps_truth"] +
             df["wheel_fr_rps_meas"] - df["wheel_fr_rps_truth"] +
             df["wheel_rl_rps_meas"] - df["wheel_rl_rps_truth"] +
             df["wheel_rr_rps_meas"] - df["wheel_rr_rps_truth"]) / 4.0
        ).values

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Sensor Noise Frequency Spectrum",
                     fontsize=16, fontweight="bold")

        for ax, err, ylabel, title in [
            (axes[0, 0], v_err,     "PSD (V/√Hz)",    "Voltage Error Spectrum"),
            (axes[0, 1], i_err,     "PSD (A/√Hz)",    "Current Error Spectrum"),
            (axes[1, 0], soc_err,   "PSD (%/√Hz)",    "SOC Error Spectrum"),
            (axes[1, 1], wheel_err, "PSD (rps/√Hz)",  "Wheel Speed Error Spectrum"),
        ]:
            psd = 2.0 / N * np.abs(fft(err)[0:N // 2])
            ax.semilogy(xf, psd)
            ax.set_xlabel("Frequency (Hz)")
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.grid(True)
            ax.set_xlim([0, 5])

        plt.tight_layout()
        return fig
