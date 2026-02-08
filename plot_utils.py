#!/usr/bin/env python3
"""
plot_utils.py - Shared plotting utilities for vehicle dynamics visualization

Provides common functions for data validation, formatting, and shared plot elements.
"""
import pandas as pd
import numpy as np


def check_columns(df, required_cols):
    """
    Check if dataframe has all required columns.

    Args:
        df: pandas DataFrame
        required_cols: list of column names

    Returns:
        tuple: (has_all, missing_cols)
    """
    missing = [c for c in required_cols if c not in df.columns]
    return (len(missing) == 0, missing)


def get_battery_columns(df):
    """
    Detect which battery column naming convention is used.

    Returns:
        tuple: (has_truth_meas, soc_col, v_col, i_col)
    """
    has_truth_meas = 'batt_soc_truth' in df.columns

    if has_truth_meas:
        soc_col = 'batt_soc_truth'
        v_col = 'batt_v_truth'
        i_col = 'batt_i_truth'
    else:
        soc_col = 'batt_soc_pct'
        v_col = 'batt_v'
        i_col = 'batt_i'

    return has_truth_meas, soc_col, v_col, i_col


def has_3dof_data(df):
    """
    Check if dataframe contains 3-DOF dynamics data.

    Returns:
        bool: True if 3-DOF data is present
    """
    required_3dof = ["vy_mps", "yaw_rate_radps", "a_lat_mps2"]
    has_all, _ = check_columns(df, required_3dof)
    return has_all


def has_tire_data(df):
    """
    Check if dataframe contains tire dynamics data.

    Returns:
        bool: True if tire data is present
    """
    required_tire = ["Fx_fl", "Fy_fl", "Fz_fl", "sigma_x_fl"]
    has_all, _ = check_columns(df, required_tire)
    return has_all


def has_slip_angle_data(df):
    """
    Check if dataframe contains slip angle data.

    Returns:
        bool: True if slip angle data is present
    """
    required = ["alpha_fl", "alpha_fr", "alpha_rl", "alpha_rr"]
    has_all, _ = check_columns(df, required)
    return has_all


def format_time_axis(ax, df):
    """Add consistent time axis formatting."""
    ax.set_xlabel("t (s)")
    ax.grid(True)


def add_zero_line(ax, axis='y', **kwargs):
    """Add a zero reference line."""
    default_style = {'color': 'k', 'linestyle': '--', 'linewidth': 0.5}
    default_style.update(kwargs)

    if axis == 'y':
        ax.axhline(y=0, **default_style)
    elif axis == 'x':
        ax.axvline(x=0, **default_style)


def symmetric_limits(ax, data, margin=1.1, axis='y'):
    """Set symmetric axis limits around zero."""
    data_min = float(data.min())
    data_max = float(data.max())
    data_abs = max(abs(data_min), abs(data_max))

    if data_abs > 1e-9:
        limit = margin * data_abs
        if axis == 'y':
            ax.set_ylim(-limit, limit)
        elif axis == 'x':
            ax.set_xlim(-limit, limit)


def deg_to_rad(deg):
    """Convert degrees to radians."""
    return deg * np.pi / 180.0


def rad_to_deg(rad):
    """Convert radians to degrees."""
    return rad * 180.0 / np.pi


def compute_sideslip_angle(df):
    """
    Compute vehicle sideslip angle (beta) from velocities.

    beta = atan2(vy, vx)

    Args:
        df: DataFrame with 'v_mps' and 'vy_mps'

    Returns:
        Series: sideslip angle in degrees
    """
    if 'vy_mps' not in df.columns or 'v_mps' not in df.columns:
        return None

    # Use vx approximation if not directly available
    vx = df['v_mps']  # v_mps is primarily longitudinal
    vy = df['vy_mps']

    beta_rad = np.arctan2(vy, vx)
    return rad_to_deg(beta_rad)


def compute_total_slip(sigma_x, sigma_y):
    """
    Compute total slip magnitude from longitudinal and lateral components.

    Args:
        sigma_x: longitudinal slip ratio
        sigma_y: lateral slip ratio

    Returns:
        total slip magnitude
    """
    return np.sqrt(sigma_x**2 + sigma_y**2)


def print_summary(df):
    """Print simulation summary statistics."""
    _, soc_col, _, _ = get_battery_columns(df)

    print(f"\n{'='*60}")
    print("SIMULATION SUMMARY")
    print(f"{'='*60}")
    print(f"Duration:     {df['t_s'].max():.1f} s")
    print(f"Distance:     {np.sqrt(df['x_m'].iloc[-1]**2 + df['y_m'].iloc[-1]**2):.1f} m")
    print(f"Max Speed:    {df['v_mps'].max():.1f} m/s ({df['v_mps'].max() * 3.6:.1f} km/h)")
    print(f"Final Speed:  {df['v_mps'].iloc[-1]:.1f} m/s")
    print(f"SOC Change:   {df[soc_col].iloc[0]:.1f}% → {df[soc_col].iloc[-1]:.1f}%")

    # 3-DOF specific stats
    if has_3dof_data(df):
        print(f"\n3-DOF Dynamics:")
        print(f"Max Lat Accel: {df['a_lat_mps2'].max():.2f} m/s² ({df['a_lat_mps2'].max()/9.81:.2f} g)")
        print(f"Max Yaw Rate:  {df['yaw_rate_radps'].max():.2f} rad/s ({rad_to_deg(df['yaw_rate_radps'].max()):.1f} deg/s)")
        print(f"Max Lat Vel:   {df['vy_mps'].max():.2f} m/s")

        beta = compute_sideslip_angle(df)
        if beta is not None:
            print(f"Max Sideslip:  {beta.max():.2f} deg")

    # Tire dynamics stats
    if has_tire_data(df):
        dynamic_mode = df.get("dynamic_model", pd.Series([0])).iloc[0]
        print(f"\nTire Dynamics: {'ENABLED' if dynamic_mode else 'DISABLED'}")
        print(f"Surface μ:    {df['surface_mu'].mean():.2f}")

        Fx_total = df["Fx_rl"] + df["Fx_rr"]
        print(f"Max Fx_rear:  {Fx_total.max()/1000:.1f} kN")

        lambda_min = df[["lambda_rl", "lambda_rr"]].min().min()
        if lambda_min < 1.0:
            print(f"⚠️  TRACTION LIMITED: λ_min = {lambda_min:.2f}")

    print(f"{'='*60}\n")


def validate_dataframe(df, csv_path):
    """
    Validate that dataframe has minimum required columns.

    Raises:
        RuntimeError: if required columns are missing
    """
    _, soc_col, v_col, i_col = get_battery_columns(df)

    required = ["t_s", "x_m", "y_m", "yaw_deg", "v_mps", "steer_deg", "motor_nm",
                "brake_pct", soc_col, v_col, i_col, "motor_power_kW",
                "regen_power_kW", "brake_force_kN"]

    has_all, missing = check_columns(df, required)
    if not has_all:
        raise RuntimeError(
            f"Missing columns in {csv_path}: {missing}.\n"
            f"Got: {list(df.columns)}"
        )

    if len(df) < 2:
        print(f"WARNING: Only {len(df)} rows in CSV. Run simulation longer!")
        return False

    return True
