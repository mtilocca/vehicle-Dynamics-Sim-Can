"""Shared data-validation, formatting, and math utilities."""
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt


class DataUtils:
    """Static utility methods.  No instance state — call as DataUtils.method()."""

    # ── column detection ───────────────────────────────────────────────────

    @staticmethod
    def check_columns(df: pd.DataFrame,
                      required_cols: list) -> "tuple[bool, list]":
        """Return (has_all, missing_cols)."""
        missing = [c for c in required_cols if c not in df.columns]
        return (len(missing) == 0, missing)

    @staticmethod
    def get_battery_columns(df: pd.DataFrame) -> "tuple[bool, str, str, str]":
        """
        Detect battery naming convention.
        Returns (has_truth_meas, soc_col, v_col, i_col).
        """
        has_truth_meas = "batt_soc_truth" in df.columns
        if has_truth_meas:
            return True, "batt_soc_truth", "batt_v_truth", "batt_i_truth"
        return False, "batt_soc_pct", "batt_v", "batt_i"

    @staticmethod
    def has_3dof_data(df: pd.DataFrame) -> bool:
        """True if vy_mps, yaw_rate_radps, a_lat_mps2 are present."""
        ok, _ = DataUtils.check_columns(df, ["vy_mps", "yaw_rate_radps", "a_lat_mps2"])
        return ok

    @staticmethod
    def has_tire_data(df: pd.DataFrame) -> bool:
        """True if Fx_fl, Fy_fl, Fz_fl, sigma_x_fl are present."""
        ok, _ = DataUtils.check_columns(df, ["Fx_fl", "Fy_fl", "Fz_fl", "sigma_x_fl"])
        return ok

    @staticmethod
    def has_slip_angle_data(df: pd.DataFrame) -> bool:
        """True if alpha_fl/fr/rl/rr are present."""
        ok, _ = DataUtils.check_columns(
            df, ["alpha_fl", "alpha_fr", "alpha_rl", "alpha_rr"])
        return ok

    # ── validation & reporting ─────────────────────────────────────────────

    @staticmethod
    def validate_dataframe(df: pd.DataFrame, csv_path: str) -> bool:
        """
        Validate minimum required columns exist and row count >= 2.
        Raises RuntimeError on missing columns; returns False (with warning)
        if row count < 2.
        """
        _, soc_col, v_col, i_col = DataUtils.get_battery_columns(df)
        required = [
            "t_s", "x_m", "y_m", "yaw_deg", "v_mps", "steer_deg", "motor_nm",
            "brake_pct", soc_col, v_col, i_col,
            "motor_power_kW", "regen_power_kW", "brake_force_kN",
        ]
        has_all, missing = DataUtils.check_columns(df, required)
        if not has_all:
            raise RuntimeError(
                f"Missing columns in {csv_path}: {missing}.\n"
                f"Got: {list(df.columns)}"
            )
        if len(df) < 2:
            print(f"WARNING: Only {len(df)} rows in CSV. Run simulation longer!")
            return False
        return True

    @staticmethod
    def print_summary(df: pd.DataFrame) -> None:
        """Print simulation summary statistics."""
        _, soc_col, _, _ = DataUtils.get_battery_columns(df)
        print(f"\n{'='*60}")
        print("SIMULATION SUMMARY")
        print(f"{'='*60}")
        print(f"Duration:     {df['t_s'].max():.1f} s")
        print(f"Distance:     "
              f"{np.sqrt(df['x_m'].iloc[-1]**2 + df['y_m'].iloc[-1]**2):.1f} m")
        print(f"Max Speed:    "
              f"{df['v_mps'].max():.1f} m/s ({df['v_mps'].max() * 3.6:.1f} km/h)")
        print(f"Final Speed:  {df['v_mps'].iloc[-1]:.1f} m/s")
        print(f"SOC Change:   "
              f"{df[soc_col].iloc[0]:.1f}% → {df[soc_col].iloc[-1]:.1f}%")

        if DataUtils.has_3dof_data(df):
            print(f"\n3-DOF Dynamics:")
            print(f"  Max Lat Accel: "
                  f"{df['a_lat_mps2'].max():.2f} m/s² "
                  f"({df['a_lat_mps2'].max() / 9.81:.2f} g)")
            print(f"  Max Yaw Rate:  "
                  f"{df['yaw_rate_radps'].max():.2f} rad/s "
                  f"({DataUtils.rad_to_deg(df['yaw_rate_radps'].max()):.1f} deg/s)")
            print(f"  Max Lat Vel:   {df['vy_mps'].max():.2f} m/s")
            beta = DataUtils.compute_sideslip_angle(df)
            if beta is not None:
                print(f"  Max Sideslip:  {beta.max():.2f} deg")

        if DataUtils.has_tire_data(df):
            dynamic_mode = df.get("dynamic_model", pd.Series([0])).iloc[0]
            print(f"\nTire Dynamics: {'ENABLED' if dynamic_mode else 'DISABLED'}")
            print(f"  Surface μ:    {df['surface_mu'].mean():.2f}")
            Fx_total = df["Fx_rl"] + df["Fx_rr"]
            print(f"  Max Fx_rear:  {Fx_total.max() / 1000:.1f} kN")
            lambda_min = df[["lambda_rl", "lambda_rr"]].min().min()
            if lambda_min < 1.0:
                print(f"  ⚠  TRACTION LIMITED: λ_min = {lambda_min:.2f}")

        print(f"{'='*60}\n")

    # ── plotting helpers ───────────────────────────────────────────────────

    @staticmethod
    def format_time_axis(ax, df: pd.DataFrame) -> None:
        """Set xlabel='t (s)' and enable grid."""
        ax.set_xlabel("t (s)")
        ax.grid(True)

    @staticmethod
    def add_zero_line(ax, axis: str = "y", **kwargs) -> None:
        """Add a zero reference line.  axis: 'x', 'y', or 'both'."""
        style = {"color": "k", "linestyle": "--", "linewidth": 0.5}
        style.update(kwargs)
        if axis in ("y", "both"):
            ax.axhline(y=0, **style)
        if axis in ("x", "both"):
            ax.axvline(x=0, **style)

    @staticmethod
    def symmetric_limits(ax, data, margin: float = 1.1,
                         axis: str = "y") -> None:
        """Set symmetric ±(margin × max(|data|)) limits around zero."""
        data_abs = max(abs(float(data.min())), abs(float(data.max())))
        if data_abs > 1e-9:
            limit = margin * data_abs
            if axis == "y":
                ax.set_ylim(-limit, limit)
            elif axis == "x":
                ax.set_xlim(-limit, limit)

    # ── unit conversion ────────────────────────────────────────────────────

    @staticmethod
    def deg_to_rad(deg):
        """Degrees → radians."""
        return deg * np.pi / 180.0

    @staticmethod
    def rad_to_deg(rad):
        """Radians → degrees."""
        return rad * 180.0 / np.pi

    # ── kinematics ─────────────────────────────────────────────────────────

    @staticmethod
    def compute_sideslip_angle(df: pd.DataFrame) -> "pd.Series | None":
        """beta = atan2(vy_mps, v_mps) in degrees.  None if columns absent."""
        if "vy_mps" not in df.columns or "v_mps" not in df.columns:
            return None
        return DataUtils.rad_to_deg(np.arctan2(df["vy_mps"], df["v_mps"]))

    @staticmethod
    def compute_total_slip(sigma_x, sigma_y) -> np.ndarray:
        """Combined slip magnitude √(σx² + σy²)."""
        return np.sqrt(sigma_x**2 + sigma_y**2)

    # ── signal processing ──────────────────────────────────────────────────

    @staticmethod
    def lowpass_filter(data: np.ndarray, cutoff: float = 0.05,
                       fs: float = 100.0, order: int = 4) -> np.ndarray:
        """4th-order Butterworth low-pass filter (bias drift extraction)."""
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype="low", analog=False)
        return filtfilt(b, a, data)
