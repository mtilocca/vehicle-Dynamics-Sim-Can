# Vehicle Dynamics Simulation - Plotting Guide

## Overview

The plotting system is now modularized into three files for better organization and scalability:

```
sim_plotter.py    # Main plotter (Figures 1 & 2) - Run this as before!
plot_3dof.py      # Advanced 3-DOF analysis (Figures 3, 4, 5)
plot_utils.py     # Shared utilities and helpers
```

## Quick Start

**Run as before** - Nothing has changed for basic usage:

```bash
python3 sim_plotter.py              # Auto-detects and plots all available data
python3 sim_plotter.py sim_out.csv  # Specify CSV file
python3 sim_plotter.py --basic      # Only Figures 1 & 2 (skip advanced plots)
```

## What Gets Plotted

The plotter automatically detects what data is available and generates the appropriate figures:

### Figure 1: Vehicle Dynamics & Battery (Always)
- Trajectory (x vs y)
- Speed & acceleration
- Steering (cmd vs actual) & yaw
- Inputs (motor torque, brake force, brake pedal)
- Battery SOC, voltage, current
- Motor & regen power
- Battery characteristic curves

### Figure 2: Tire Dynamics (If tire data available)
- Longitudinal forces (Fx) per wheel
- Lateral forces (Fy) per wheel
- Normal loads (Fz) with load transfer
- Slip ratios (σx, σy)
- Friction utilization (λ)
- Friction circles (front & rear)
- Load transfer vs acceleration
- Slip-force characteristics

### Figure 3: 3-DOF Lateral Dynamics (If 3-DOF data available)
- Lateral velocity (vy)
- Yaw rate (rad/s and deg/s)
- Lateral acceleration (m/s² and g-force)
- Vehicle sideslip angle (β)
- Front/rear slip angles per tire
- Phase portraits (vy vs yaw_rate)
- Yaw rate response to steering
- Understeer analysis
- Trajectory with velocity vectors

### Figure 4: Wheel Torque Distribution (If torque data available)
- Drive torques (RL, RR)
- Total drive torque
- Drive torque asymmetry
- Brake torques (all wheels)
- Front/rear brake balance
- Wheel angular velocities (RPM)
- Wheel speed asymmetry
- Torque vs speed operating points
- Mechanical power (τ × ω)

### Figure 5: Combined Slip Analysis (If slip data available)
- Longitudinal slip ratios (σx)
- Lateral slip ratios (σy)
- Total slip magnitude √(σx² + σy²)
- Combined slip diagrams (σx vs σy)
- Force vectors (Fx vs Fy)
- Friction circles with operating points
- Slip-force correlations
- Friction utilization vs total slip

## Advanced Usage

### Run Only 3-DOF Plots
```bash
python3 plot_3dof.py                # Just Figures 3, 4, 5
python3 plot_3dof.py sim_out.csv
```

### Skip Advanced Plots
```bash
python3 sim_plotter.py --basic      # Only Figures 1 & 2
```

## Data Requirements

### Minimum Required (Figure 1)
- `t_s`, `x_m`, `y_m`, `yaw_deg`, `v_mps`
- `steer_deg`, `motor_nm`, `brake_pct`
- `batt_soc_pct`, `batt_v`, `batt_i` (or `_truth` variants)
- `motor_power_kW`, `regen_power_kW`, `brake_force_kN`

### For Tire Dynamics (Figure 2)
- `Fx_fl`, `Fx_fr`, `Fx_rl`, `Fx_rr` (forces in N)
- `Fy_fl`, `Fy_fr`, `Fy_rl`, `Fy_rr`
- `Fz_fl`, `Fz_fr`, `Fz_rl`, `Fz_rr`
- `sigma_x_*`, `sigma_y_*` (slip ratios)
- `lambda_*` (friction utilization)
- `surface_mu`

### For 3-DOF Lateral (Figure 3)
- `vy_mps` (lateral velocity)
- `yaw_rate_radps`
- `a_lat_mps2`
- `alpha_fl`, `alpha_fr`, `alpha_rl`, `alpha_rr` (slip angles)

### For Wheel Torques (Figure 4)
- `tau_drive_rl_nm`, `tau_drive_rr_nm`
- `tau_brake_fl_nm`, `tau_brake_fr_nm`, `tau_brake_rl_nm`, `tau_brake_rr_nm`
- `omega_fl_radps`, `omega_fr_radps`, `omega_rl_radps`, `omega_rr_radps`

## Module Details

### `plot_utils.py`
Shared utilities:
- `check_columns()` - Validate dataframe columns
- `get_battery_columns()` - Detect battery naming convention
- `has_3dof_data()` - Check for 3-DOF data
- `has_tire_data()` - Check for tire data
- `compute_sideslip_angle()` - Calculate β from velocities
- `compute_total_slip()` - Calculate total slip magnitude
- `print_summary()` - Comprehensive simulation summary
- `validate_dataframe()` - Data validation

### `plot_3dof.py`
Advanced 3-DOF plotting:
- `plot_lateral_dynamics()` - Figure 3
- `plot_wheel_torques()` - Figure 4
- `plot_combined_slip()` - Figure 5

Can be imported or run standalone.

### `sim_plotter.py`
Main plotter (backward compatible):
- `plot_vehicle_dynamics()` - Figure 1
- `plot_tire_dynamics()` - Figure 2
- Automatically calls advanced plots if available

## Examples

### Example 1: Full 3-DOF Analysis
```bash
# Run simulation with 3-DOF dynamics
./build/vehicle_sim --config configs/heavy_truck.yaml --duration 60

# Plot everything (5 figures)
python3 sim_plotter.py
```

### Example 2: Compare Two Runs
```bash
# Run baseline
./build/vehicle_sim --duration 30
mv sim_out.csv baseline.csv

# Run modified config
./build/vehicle_sim --config modified.yaml --duration 30
mv sim_out.csv modified.csv

# Plot both
python3 sim_plotter.py baseline.csv &
python3 sim_plotter.py modified.csv &
```

### Example 3: Quick Tire Check
```bash
# Just tire dynamics
python3 sim_plotter.py --basic  # Figures 1 & 2 only
```

## Physics Conventions

- **Longitudinal (x)**: +forward, -backward
- **Lateral (y)**: +left, -right
- **Yaw**: +CCW (counterclockwise), -CW (clockwise)
- **Slip Ratios**:
  - σx: +acceleration, -braking
  - σy: lateral slip (≈ slip angle for small angles)
- **Forces**:
  - Fx: +drive, -brake
  - Fy: +left, -right (opposes slip)
  - Fz: normal load (always positive)
- **Sideslip β**: atan2(vy, vx) - angle between heading and velocity

## Troubleshooting

### "Missing columns" error
Check that your simulation is logging the required data. For 3-DOF:
- Ensure `plant_state.hpp` includes `vy_mps`, `yaw_rate_radps`, `a_lat_mps2`
- Run with `--dynamic-model` flag if available

### "Advanced plotting modules not found"
Make sure all three Python files are in the same directory:
- `sim_plotter.py`
- `plot_utils.py`
- `plot_3dof.py`

### Plots look wrong
- Check units: Forces in N, torques in Nm, angles in degrees/radians
- Verify coordinate frames: tire forces should be in vehicle frame
- Look for NaN values: `df.isna().sum()`

### "Only X rows in CSV"
Simulation ran for very short time. Increase `--duration` parameter.

## Adding Custom Plots

To add your own analysis:

1. **Add to existing figure**: Edit the appropriate function in `sim_plotter.py` or `plot_3dof.py`
2. **Create new figure**: Add a new function in `plot_3dof.py`, then call it from `main()`
3. **Create new module**: Make `plot_custom.py`, import in `sim_plotter.py`

Example:
```python
def plot_custom_analysis(df):
    fig = plt.figure(figsize=(15, 10))
    # ... your plots ...
    return fig
```

Then in `sim_plotter.py` main():
```python
if "custom_data" in df.columns:
    fig_custom = plot_custom_analysis(df)
```
