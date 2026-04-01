"""Figure 11 — IMU sensor validation."""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from .base_plot import BasePlot
from .data_utils import DataUtils


class ImuValidationPlot(BasePlot):
    """3×2 IMU gyro and accelerometer validation plot (Figure 11)."""

    _REQUIRED = [
        "imu_gx_rps", "imu_gy_rps", "imu_gz_rps",
        "imu_ax_mps2", "imu_ay_mps2", "imu_az_mps2",
        "yaw_deg", "v_mps",
    ]

    @property
    def _filename(self) -> str:
        return "fig11_imu_validation"

    def plot(self, df: pd.DataFrame):
        has_all, missing = DataUtils.check_columns(df, self._REQUIRED)
        if not has_all:
            print(f"[WARN] Missing IMU columns: {missing}")
            return None

        yaw_rate_truth = np.gradient(df["yaw_deg"].values, df["t_s"].values)
        accel_x_truth = np.gradient(df["v_mps"].values, df["t_s"].values)

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle("IMU Sensor (6-DOF) - Under Development",
                     fontsize=16, fontweight="bold")

        # Gyro X — should be ~0
        axes[0, 0].plot(df["t_s"], df["imu_gx_rps"], "r-", linewidth=2)
        axes[0, 0].axhline(y=0, color="b", linestyle="--", alpha=0.5)
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel("Roll Rate (rad/s)")
        axes[0, 0].set_title("Gyro X (Should be ~0)")
        axes[0, 0].grid(True, alpha=0.3)

        # Gyro Y — should be ~0
        axes[0, 1].plot(df["t_s"], df["imu_gy_rps"], "r-", linewidth=2)
        axes[0, 1].axhline(y=0, color="b", linestyle="--", alpha=0.5)
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("Pitch Rate (rad/s)")
        axes[0, 1].set_title("Gyro Y (Should be ~0)")
        axes[0, 1].grid(True, alpha=0.3)

        # Gyro Z — yaw rate
        axes[1, 0].plot(df["t_s"], yaw_rate_truth, "b-",
                        label="Truth", linewidth=2)
        axes[1, 0].plot(df["t_s"], df["imu_gz_rps"], "r--",
                        label="Measured", alpha=0.7)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Yaw Rate (rad/s)")
        axes[1, 0].set_title("Gyro Z (Yaw Rate)")
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # Accel X — longitudinal
        axes[1, 1].plot(df["t_s"], accel_x_truth, "b-",
                        label="Truth", linewidth=2)
        axes[1, 1].plot(df["t_s"], df["imu_ax_mps2"], "r--",
                        label="Measured", alpha=0.7)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Accel X (m/s²)")
        axes[1, 1].set_title("Accel X (Longitudinal)")
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        # Accel Y — lateral
        axes[2, 0].plot(df["t_s"], df["imu_ay_mps2"], "r-", linewidth=2)
        axes[2, 0].axhline(y=0, color="b", linestyle="--", alpha=0.5)
        axes[2, 0].set_xlabel("Time (s)")
        axes[2, 0].set_ylabel("Accel Y (m/s²)")
        axes[2, 0].set_title("Accel Y (Lateral)")
        axes[2, 0].grid(True, alpha=0.3)

        # Accel Z — should be ~-9.81
        axes[2, 1].plot(df["t_s"], df["imu_az_mps2"], "r-", linewidth=2)
        axes[2, 1].axhline(y=-9.81, color="b", linestyle="--", alpha=0.5)
        axes[2, 1].set_xlabel("Time (s)")
        axes[2, 1].set_ylabel("Accel Z (m/s²)")
        axes[2, 1].set_title("Accel Z (Should be ~-9.81)")
        axes[2, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        return fig
