#!/usr/bin/env python3
"""
Sensor vs Truth Analysis
Analyzes sensor noise, bias drift, and measurement errors
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def load_data(csv_path='sim_out.csv'):
    """Load simulation CSV data"""
    df = pd.read_csv(csv_path)
    return df

def analyze_sensor_errors(df):
    """Calculate error statistics for all sensors"""
    
    errors = {}
    
    # Battery voltage error
    if 'batt_v_truth' in df.columns and 'batt_v_meas' in df.columns:
        v_err = df['batt_v_meas'] - df['batt_v_truth']
        errors['voltage'] = {
            'mean': v_err.mean(),
            'std': v_err.std(),
            'max': v_err.abs().max(),
            'rmse': np.sqrt((v_err**2).mean())
        }
    
    # Battery current error
    if 'batt_i_truth' in df.columns and 'batt_i_meas' in df.columns:
        i_err = df['batt_i_meas'] - df['batt_i_truth']
        errors['current'] = {
            'mean': i_err.mean(),
            'std': i_err.std(),
            'max': i_err.abs().max(),
            'rmse': np.sqrt((i_err**2).mean())
        }
    
    # Battery SOC error
    if 'batt_soc_truth' in df.columns and 'batt_soc_meas' in df.columns:
        soc_err = df['batt_soc_meas'] - df['batt_soc_truth']
        errors['soc'] = {
            'mean': soc_err.mean(),
            'std': soc_err.std(),
            'max': soc_err.abs().max(),
            'rmse': np.sqrt((soc_err**2).mean())
        }
    
    # Wheel speed errors (average across all wheels)
    wheel_errors = []
    for wheel in ['fl', 'fr', 'rl', 'rr']:
        truth_col = f'wheel_{wheel}_rps_truth'
        meas_col = f'wheel_{wheel}_rps_meas'
        if truth_col in df.columns and meas_col in df.columns:
            err = df[meas_col] - df[truth_col]
            wheel_errors.append(err)
    
    if wheel_errors:
        avg_wheel_err = pd.concat(wheel_errors, axis=1).mean(axis=1)
        errors['wheel_speed'] = {
            'mean': avg_wheel_err.mean(),
            'std': avg_wheel_err.std(),
            'max': avg_wheel_err.abs().max(),
            'rmse': np.sqrt((avg_wheel_err**2).mean())
        }
    
    return errors

def print_error_statistics(errors):
    """Print formatted error statistics"""
    print("\n" + "="*60)
    print("SENSOR ERROR ANALYSIS")
    print("="*60)
    
    for sensor, stats in errors.items():
        print(f"\n{sensor.upper()}:")
        print(f"  Mean error:  {stats['mean']:+.4f}")
        print(f"  Std dev:     {stats['std']:.4f}")
        print(f"  Max error:   {stats['max']:.4f}")
        print(f"  RMSE:        {stats['rmse']:.4f}")
    
    print("\n" + "="*60)

def plot_battery_comparison(df, output_dir='plots'):
    """Plot battery sensor truth vs measured with error analysis"""
    
    Path(output_dir).mkdir(exist_ok=True)
    
    fig, axes = plt.subplots(4, 2, figsize=(16, 12))
    fig.suptitle('Battery Sensor Analysis: Truth vs Measured', fontsize=16, fontweight='bold')
    
    t = df['t_s']
    
    # --- Voltage ---
    ax = axes[0, 0]
    ax.plot(t, df['batt_v_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(t, df['batt_v_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_ylabel('Voltage (V)')
    ax.set_title('Battery Voltage')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    v_err = df['batt_v_meas'] - df['batt_v_truth']
    ax.plot(t, v_err, 'k-', linewidth=1)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.fill_between(t, -v_err.std(), v_err.std(), alpha=0.2, label=f'±1σ = {v_err.std():.3f}V')
    ax.set_ylabel('Error (V)')
    ax.set_title('Voltage Error (Measured - Truth)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- Current ---
    ax = axes[1, 0]
    ax.plot(t, df['batt_i_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(t, df['batt_i_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_ylabel('Current (A)')
    ax.set_title('Battery Current')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 1]
    i_err = df['batt_i_meas'] - df['batt_i_truth']
    ax.plot(t, i_err, 'k-', linewidth=1)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.fill_between(t, -i_err.std(), i_err.std(), alpha=0.2, label=f'±1σ = {i_err.std():.3f}A')
    ax.set_ylabel('Error (A)')
    ax.set_title('Current Error')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- SOC ---
    ax = axes[2, 0]
    ax.plot(t, df['batt_soc_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(t, df['batt_soc_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_ylabel('SOC (%)')
    ax.set_title('Battery State of Charge')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[2, 1]
    soc_err = df['batt_soc_meas'] - df['batt_soc_truth']
    ax.plot(t, soc_err, 'k-', linewidth=1)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.fill_between(t, -soc_err.std(), soc_err.std(), alpha=0.2, label=f'±1σ = {soc_err.std():.3f}%')
    ax.set_ylabel('Error (%)')
    ax.set_title('SOC Error')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- Error Histogram ---
    ax = axes[3, 0]
    ax.hist(v_err, bins=50, alpha=0.6, label='Voltage', color='blue')
    ax.hist(i_err, bins=50, alpha=0.6, label='Current', color='red')
    ax.set_xlabel('Error')
    ax.set_ylabel('Count')
    ax.set_title('Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- Error vs Time (all sensors) ---
    ax = axes[3, 1]
    ax.plot(t, v_err / df['batt_v_truth'].mean() * 100, label='Voltage (%)', alpha=0.7)
    ax.plot(t, i_err / df['batt_i_truth'].abs().mean() * 100, label='Current (%)', alpha=0.7)
    ax.plot(t, soc_err, label='SOC (%)', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Relative Error (%)')
    ax.set_title('Normalized Errors Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/battery_sensor_analysis.png', dpi=150, bbox_inches='tight')
    print(f"Saved: {output_dir}/battery_sensor_analysis.png")
    plt.close()

def plot_wheel_speed_comparison(df, output_dir='plots'):
    """Plot wheel speed sensor truth vs measured"""
    
    Path(output_dir).mkdir(exist_ok=True)
    
    fig, axes = plt.subplots(4, 2, figsize=(16, 12))
    fig.suptitle('Wheel Speed Sensor Analysis: Truth vs Measured', fontsize=16, fontweight='bold')
    
    t = df['t_s']
    wheels = ['fl', 'fr', 'rl', 'rr']
    wheel_names = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
    
    for i, (wheel, name) in enumerate(zip(wheels, wheel_names)):
        truth_col = f'wheel_{wheel}_rps_truth'
        meas_col = f'wheel_{wheel}_rps_meas'
        
        if truth_col not in df.columns or meas_col not in df.columns:
            continue
        
        # Truth vs Measured
        ax = axes[i, 0]
        ax.plot(t, df[truth_col], 'b-', label='Truth', linewidth=2)
        ax.plot(t, df[meas_col], 'r--', label='Measured', alpha=0.7)
        ax.set_ylabel('Speed (rps)')
        ax.set_title(f'{name} Wheel Speed')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Error
        ax = axes[i, 1]
        err = df[meas_col] - df[truth_col]
        ax.plot(t, err, 'k-', linewidth=1)
        ax.axhline(0, color='r', linestyle='--', alpha=0.5)
        ax.fill_between(t, -err.std(), err.std(), alpha=0.2, label=f'±1σ = {err.std():.4f} rps')
        ax.set_ylabel('Error (rps)')
        ax.set_title(f'{name} Error')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    for ax in axes.flatten():
        if ax.lines:  # Only label if plot has data
            ax.set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/wheel_sensor_analysis.png', dpi=150, bbox_inches='tight')
    print(f"Saved: {output_dir}/wheel_sensor_analysis.png")
    plt.close()

def plot_bias_drift_analysis(df, output_dir='plots'):
    """Analyze sensor bias drift over time (low-frequency component)"""
    
    Path(output_dir).mkdir(exist_ok=True)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sensor Bias Drift Analysis (Low-Pass Filtered)', fontsize=16, fontweight='bold')
    
    t = df['t_s']
    
    # Simple moving average filter to extract bias (low-frequency component)
    window = min(100, len(df) // 10)  # 100 samples or 10% of data
    
    # --- Voltage Bias ---
    ax = axes[0, 0]
    v_err = df['batt_v_meas'] - df['batt_v_truth']
    v_bias = v_err.rolling(window=window, center=True).mean()
    ax.plot(t, v_err, 'lightgray', alpha=0.3, label='Instantaneous Error')
    ax.plot(t, v_bias, 'b-', linewidth=2, label='Bias (Slow Drift)')
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Voltage Error (V)')
    ax.set_title('Voltage Sensor Bias Drift')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- Current Bias ---
    ax = axes[0, 1]
    i_err = df['batt_i_meas'] - df['batt_i_truth']
    i_bias = i_err.rolling(window=window, center=True).mean()
    ax.plot(t, i_err, 'lightgray', alpha=0.3, label='Instantaneous Error')
    ax.plot(t, i_bias, 'r-', linewidth=2, label='Bias (Slow Drift)')
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Current Error (A)')
    ax.set_title('Current Sensor Bias Drift')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- SOC Bias ---
    ax = axes[1, 0]
    soc_err = df['batt_soc_meas'] - df['batt_soc_truth']
    soc_bias = soc_err.rolling(window=window, center=True).mean()
    ax.plot(t, soc_err, 'lightgray', alpha=0.3, label='Instantaneous Error')
    ax.plot(t, soc_bias, 'g-', linewidth=2, label='Bias (Slow Drift)')
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    ax.set_ylabel('SOC Error (%)')
    ax.set_title('SOC Estimation Bias Drift')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # --- Combined Bias Drift ---
    ax = axes[1, 1]
    ax.plot(t, v_bias / df['batt_v_truth'].mean() * 100, label='Voltage (%)', linewidth=2)
    ax.plot(t, i_bias / df['batt_i_truth'].abs().mean() * 100, label='Current (%)', linewidth=2)
    ax.plot(t, soc_bias, label='SOC (%)', linewidth=2)
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Normalized Bias (%)')
    ax.set_title('All Sensor Bias Drift (Normalized)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    for ax in axes.flatten():
        ax.set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/sensor_bias_drift.png', dpi=150, bbox_inches='tight')
    print(f"Saved: {output_dir}/sensor_bias_drift.png")
    plt.close()

def plot_noise_spectrum(df, output_dir='plots'):
    """Analyze frequency content of sensor noise (FFT)"""
    
    Path(output_dir).mkdir(exist_ok=True)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sensor Noise Frequency Spectrum', fontsize=16, fontweight='bold')
    
    dt = df['t_s'].iloc[1] - df['t_s'].iloc[0]
    fs = 1.0 / dt  # Sampling frequency
    
    def plot_spectrum(ax, error, title, ylabel):
        # Compute FFT
        n = len(error)
        fft = np.fft.rfft(error)
        freq = np.fft.rfftfreq(n, dt)
        psd = np.abs(fft)**2 / n
        
        # Plot
        ax.semilogy(freq, psd)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3, which='both')
        ax.set_xlim(0, fs/2)
    
    # Voltage
    v_err = df['batt_v_meas'] - df['batt_v_truth']
    plot_spectrum(axes[0, 0], v_err, 'Voltage Error Spectrum', 'PSD (V²/Hz)')
    
    # Current
    i_err = df['batt_i_meas'] - df['batt_i_truth']
    plot_spectrum(axes[0, 1], i_err, 'Current Error Spectrum', 'PSD (A²/Hz)')
    
    # SOC
    soc_err = df['batt_soc_meas'] - df['batt_soc_truth']
    plot_spectrum(axes[1, 0], soc_err, 'SOC Error Spectrum', 'PSD (%²/Hz)')
    
    # Wheel speed (average)
    wheel_errs = []
    for wheel in ['fl', 'fr', 'rl', 'rr']:
        truth_col = f'wheel_{wheel}_rps_truth'
        meas_col = f'wheel_{wheel}_rps_meas'
        if truth_col in df.columns and meas_col in df.columns:
            wheel_errs.append(df[meas_col] - df[truth_col])
    
    if wheel_errs:
        avg_wheel_err = pd.concat(wheel_errs, axis=1).mean(axis=1)
        plot_spectrum(axes[1, 1], avg_wheel_err, 'Wheel Speed Error Spectrum', 'PSD (rps²/Hz)')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/sensor_noise_spectrum.png', dpi=150, bbox_inches='tight')
    print(f"Saved: {output_dir}/sensor_noise_spectrum.png")
    plt.close()

def main():
    """Main analysis script"""
    
    # Load data
    print("Loading simulation data...")
    df = load_data('sim_out.csv')
    print(f"Loaded {len(df)} timesteps ({df['t_s'].iloc[-1]:.1f} seconds)")
    
    # Analyze errors
    errors = analyze_sensor_errors(df)
    print_error_statistics(errors)
    
    # Generate plots
    print("\nGenerating analysis plots...")
    plot_battery_comparison(df)
    plot_wheel_speed_comparison(df)
    plot_bias_drift_analysis(df)
    plot_noise_spectrum(df)
    
    print("\n✓ Analysis complete! Check the 'plots/' directory.")

if __name__ == '__main__':
    main()