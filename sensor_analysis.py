#!/usr/bin/env python3
"""
Complete Sensor Suite Analysis - ENHANCED VERSION
Analyzes all sensors: Battery, Wheels, IMU, GNSS, Radar
Detailed error analysis for Battery & Wheel sensors (most accurate)
Simple validation for IMU/GNSS/Radar (still under development)
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
from scipy.signal import butter, filtfilt
import sys
import os

def lowpass_filter(data, cutoff=0.05, fs=100, order=4):
    """Low-pass Butterworth filter to extract bias drift"""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def analyze_complete_sensors(csv_file):
    """Analyze complete sensor suite with detailed error analysis"""
    
    # Load data
    print(f"Loading {csv_file}...")
    df = pd.read_csv(csv_file)
    print(f"Loaded {len(df)} timesteps ({df['t_s'].iloc[-1]:.1f} seconds)\n")
    
    # Create output directory
    os.makedirs('plots', exist_ok=True)
    
    # Calculate errors
    v_err = df['batt_v_meas'] - df['batt_v_truth']
    i_err = df['batt_i_meas'] - df['batt_i_truth']
    soc_err = df['batt_soc_meas'] - df['batt_soc_truth']
    
    v_rmse = np.sqrt(np.mean(v_err**2))
    i_rmse = np.sqrt(np.mean(i_err**2))
    soc_rmse = np.sqrt(np.mean(soc_err**2))
    
    dt = df['t_s'].iloc[1] - df['t_s'].iloc[0]
    fs = 1.0 / dt
    
    # ========================================
    # Figure 1: Battery & Wheel Sensors Overview
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Battery & Wheel Speed Sensors', fontsize=16, fontweight='bold')
    
    ax = axes[0, 0]
    ax.plot(df['t_s'], df['batt_soc_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['batt_soc_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('SOC (%)')
    ax.set_title('Battery State of Charge')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    ax.plot(df['t_s'], df['batt_v_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['batt_v_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Voltage (V)')
    ax.set_title('Battery Voltage')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 0]
    ax.plot(df['t_s'], df['batt_i_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['batt_i_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Current (A)')
    ax.set_title('Battery Current')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 1]
    ax.plot(df['t_s'], df['wheel_fl_rps_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['wheel_fl_rps_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Wheel Speed (rps)')
    ax.set_title('Front-Left Wheel Speed')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/battery_wheel_sensors.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/battery_wheel_sensors.png")
    
    # ========================================
    # Figure 2: Battery Sensor Detailed Analysis
    # ========================================
    fig, axes = plt.subplots(3, 3, figsize=(18, 12))
    fig.suptitle('Battery Sensor Analysis: Truth vs Measured', fontsize=16, fontweight='bold')
    
    # Voltage
    axes[0,0].plot(df['t_s'], df['batt_v_truth'], label='Truth', color='blue')
    axes[0,0].plot(df['t_s'], df['batt_v_meas'], label='Measured', color='red', linestyle='--', alpha=0.7)
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Voltage (V)')
    axes[0,0].set_title('Battery Voltage')
    axes[0,0].legend()
    axes[0,0].grid(True)
    
    axes[0,1].plot(df['t_s'], v_err, color='black')
    axes[0,1].axhline(y=0, color='red', linestyle='--', linewidth=0.5)
    axes[0,1].fill_between(df['t_s'], -v_rmse, v_rmse, alpha=0.2, color='blue', label=f'±1σ = {v_rmse:.4f}V')
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Error (V)')
    axes[0,1].set_title('Voltage Error (Measured - Truth)')
    axes[0,1].legend()
    axes[0,1].grid(True)
    
    # Current
    axes[1,0].plot(df['t_s'], df['batt_i_truth'], label='Truth', color='blue')
    axes[1,0].plot(df['t_s'], df['batt_i_meas'], label='Measured', color='red', linestyle='--', alpha=0.7)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Current (A)')
    axes[1,0].set_title('Battery Current')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    axes[1,1].plot(df['t_s'], i_err, color='black')
    axes[1,1].axhline(y=0, color='red', linestyle='--', linewidth=0.5)
    axes[1,1].fill_between(df['t_s'], -i_rmse, i_rmse, alpha=0.2, color='blue', label=f'±1σ = {i_rmse:.3f}A')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Error (A)')
    axes[1,1].set_title('Current Error')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    # SOC
    axes[2,0].plot(df['t_s'], df['batt_soc_truth'], label='Truth', color='blue')
    axes[2,0].plot(df['t_s'], df['batt_soc_meas'], label='Measured', color='red', linestyle='--', alpha=0.7)
    axes[2,0].set_xlabel('Time (s)')
    axes[2,0].set_ylabel('SOC (%)')
    axes[2,0].set_title('Battery State of Charge')
    axes[2,0].legend()
    axes[2,0].grid(True)
    
    axes[2,1].plot(df['t_s'], soc_err, color='black')
    axes[2,1].axhline(y=0, color='red', linestyle='--', linewidth=0.5)
    axes[2,1].fill_between(df['t_s'], -soc_rmse, soc_rmse, alpha=0.2, color='blue', label=f'±1σ = {soc_rmse:.4f}%')
    axes[2,1].set_xlabel('Time (s)')
    axes[2,1].set_ylabel('Error (%)')
    axes[2,1].set_title('SOC Error')
    axes[2,1].legend()
    axes[2,1].grid(True)
    
    # Error distributions
    axes[0,2].hist(v_err, bins=30, color='blue', alpha=0.7, edgecolor='black')
    axes[0,2].axvline(x=0, color='red', linestyle='--', linewidth=2)
    axes[0,2].set_xlabel('Error (V)')
    axes[0,2].set_ylabel('Count')
    axes[0,2].set_title('Voltage')
    axes[0,2].grid(True)
    
    axes[1,2].hist(i_err, bins=30, color='red', alpha=0.7, edgecolor='black')
    axes[1,2].axvline(x=0, color='red', linestyle='--', linewidth=2)
    axes[1,2].set_xlabel('Error (A)')
    axes[1,2].set_ylabel('Count')
    axes[1,2].set_title('Current')
    axes[1,2].grid(True)
    
    # Normalized errors
    v_err_norm = (v_err / v_rmse) * 100
    i_err_norm = (i_err / i_rmse) * 100
    soc_err_norm = (soc_err / soc_rmse) * 100
    
    axes[2,2].plot(df['t_s'], v_err_norm, label='Voltage (%)', alpha=0.7)
    axes[2,2].plot(df['t_s'], i_err_norm, label='Current (%)', alpha=0.7)
    axes[2,2].plot(df['t_s'], soc_err_norm, label='SOC (%)', alpha=0.7)
    axes[2,2].axhline(y=0, color='black', linestyle='--', linewidth=0.5)
    axes[2,2].set_xlabel('Time (s)')
    axes[2,2].set_ylabel('Relative Error (%)')
    axes[2,2].set_title('Normalized Errors Over Time')
    axes[2,2].legend()
    axes[2,2].grid(True)
    
    plt.tight_layout()
    plt.savefig('plots/battery_sensor_analysis.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/battery_sensor_analysis.png")
    
    # ========================================
    # Figure 3: Wheel Sensor Analysis
    # ========================================
    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    fig.suptitle('Wheel Speed Sensor Analysis: All 4 Wheels', fontsize=16, fontweight='bold')
    
    wheels = [('fl', 'Front Left'), ('fr', 'Front Right'), ('rl', 'Rear Left'), ('rr', 'Rear Right')]
    
    for idx, (wheel, title) in enumerate(wheels):
        truth_col = f'wheel_{wheel}_rps_truth'
        meas_col = f'wheel_{wheel}_rps_meas'
        
        err = df[meas_col] - df[truth_col]
        rmse = np.sqrt(np.mean(err**2))
        
        axes[idx, 0].plot(df['t_s'], df[truth_col], label='Truth', color='blue')
        axes[idx, 0].plot(df['t_s'], df[meas_col], label='Measured', color='red', linestyle='--', alpha=0.7)
        axes[idx, 0].set_xlabel('Time (s)')
        axes[idx, 0].set_ylabel('Speed (rps)')
        axes[idx, 0].set_title(f'{title} Wheel Speed')
        axes[idx, 0].legend()
        axes[idx, 0].grid(True)
        
        axes[idx, 1].plot(df['t_s'], err, color='black')
        axes[idx, 1].axhline(y=0, color='red', linestyle='--', linewidth=0.5)
        axes[idx, 1].fill_between(df['t_s'], -rmse, rmse, alpha=0.2, color='blue', label=f'±1σ = {rmse:.4f} rps')
        axes[idx, 1].set_xlabel('Time (s)')
        axes[idx, 1].set_ylabel('Error (rps)')
        axes[idx, 1].set_title(f'{title} Error')
        axes[idx, 1].legend()
        axes[idx, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('plots/wheel_sensor_analysis.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/wheel_sensor_analysis.png")
    
    # ========================================
    # Figure 4: Sensor Noise Spectrum
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sensor Noise Frequency Spectrum', fontsize=16, fontweight='bold')
    
    N = len(v_err)
    
    # Voltage (convert to numpy array)
    yf = fft(v_err.values)
    xf = fftfreq(N, dt)[:N//2]
    psd = 2.0/N * np.abs(yf[0:N//2])
    axes[0, 0].semilogy(xf, psd)
    axes[0, 0].set_xlabel('Frequency (Hz)')
    axes[0, 0].set_ylabel('PSD (V/√Hz)')
    axes[0, 0].set_title('Voltage Error Spectrum')
    axes[0, 0].grid(True)
    axes[0, 0].set_xlim([0, 5])
    
    # Current (convert to numpy array)
    yf = fft(i_err.values)
    psd = 2.0/N * np.abs(yf[0:N//2])
    axes[0, 1].semilogy(xf, psd)
    axes[0, 1].set_xlabel('Frequency (Hz)')
    axes[0, 1].set_ylabel('PSD (A/√Hz)')
    axes[0, 1].set_title('Current Error Spectrum')
    axes[0, 1].grid(True)
    axes[0, 1].set_xlim([0, 5])
    
    # SOC (convert to numpy array)
    yf = fft(soc_err.values)
    psd = 2.0/N * np.abs(yf[0:N//2])
    axes[1, 0].semilogy(xf, psd)
    axes[1, 0].set_xlabel('Frequency (Hz)')
    axes[1, 0].set_ylabel('PSD (%/√Hz)')
    axes[1, 0].set_title('SOC Error Spectrum')
    axes[1, 0].grid(True)
    axes[1, 0].set_xlim([0, 5])
    
    # Wheel (average) - convert to numpy array
    wheel_err = (df['wheel_fl_rps_meas'] - df['wheel_fl_rps_truth'] +
                 df['wheel_fr_rps_meas'] - df['wheel_fr_rps_truth'] +
                 df['wheel_rl_rps_meas'] - df['wheel_rl_rps_truth'] +
                 df['wheel_rr_rps_meas'] - df['wheel_rr_rps_truth']) / 4.0
    yf = fft(wheel_err.values)
    psd = 2.0/N * np.abs(yf[0:N//2])
    axes[1, 1].semilogy(xf, psd)
    axes[1, 1].set_xlabel('Frequency (Hz)')
    axes[1, 1].set_ylabel('PSD (rps/√Hz)')
    axes[1, 1].set_title('Wheel Speed Error Spectrum')
    axes[1, 1].grid(True)
    axes[1, 1].set_xlim([0, 5])
    
    plt.tight_layout()
    plt.savefig('plots/sensor_noise_spectrum.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/sensor_noise_spectrum.png")
    
    # ========================================
    # Figure 5: Sensor Bias Drift
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sensor Bias Drift Analysis (Low-Pass Filtered)', fontsize=16, fontweight='bold')
    
    # Convert to numpy arrays for filtering
    v_bias = lowpass_filter(v_err.values, cutoff=0.05, fs=fs)
    i_bias = lowpass_filter(i_err.values, cutoff=0.05, fs=fs)
    soc_bias = lowpass_filter(soc_err.values, cutoff=0.05, fs=fs)
    
    axes[0, 0].plot(df['t_s'], v_err, color='lightgray', alpha=0.5, label='Instantaneous Error')
    axes[0, 0].plot(df['t_s'], v_bias, color='blue', linewidth=2, label='Bias (Slow Drift)')
    axes[0, 0].axhline(y=0, color='black', linestyle='--', linewidth=0.5)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Voltage Error (V)')
    axes[0, 0].set_title('Voltage Sensor Bias Drift')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(df['t_s'], i_err, color='lightgray', alpha=0.5, label='Instantaneous Error')
    axes[0, 1].plot(df['t_s'], i_bias, color='red', linewidth=2, label='Bias (Slow Drift)')
    axes[0, 1].axhline(y=0, color='black', linestyle='--', linewidth=0.5)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Current Error (A)')
    axes[0, 1].set_title('Current Sensor Bias Drift')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    axes[1, 0].plot(df['t_s'], soc_err, color='lightgray', alpha=0.5, label='Instantaneous Error')
    axes[1, 0].plot(df['t_s'], soc_bias, color='green', linewidth=2, label='Bias (Slow Drift)')
    axes[1, 0].axhline(y=0, color='black', linestyle='--', linewidth=0.5)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('SOC Error (%)')
    axes[1, 0].set_title('SOC Estimation Bias Drift')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(df['t_s'], (v_bias / v_rmse) * 100, label='Voltage (%)', color='blue')
    axes[1, 1].plot(df['t_s'], (i_bias / i_rmse) * 100, label='Current (%)', color='orange')
    axes[1, 1].plot(df['t_s'], (soc_bias / soc_rmse) * 100, label='SOC (%)', color='green')
    axes[1, 1].axhline(y=0, color='black', linestyle='--', linewidth=0.5)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Normalized Bias (%)')
    axes[1, 1].set_title('All Sensor Bias Drift (Normalized)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('plots/sensor_bias_drift.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/sensor_bias_drift.png")
    
    # ========================================
    # Figure 6: IMU Sensor (Simple Validation)
    # ========================================
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('IMU Sensor (6-DOF) - Under Development', fontsize=16, fontweight='bold')
    
    # Calculate truth values
    yaw_rate_truth_rps = np.gradient(df['yaw_deg'].values, df['t_s'].values)
    accel_x_truth = np.gradient(df['v_mps'].values, df['t_s'].values)
    
    axes[0, 0].plot(df['t_s'], df['imu_gx_rps'], 'r-', linewidth=2)
    axes[0, 0].axhline(y=0, color='b', linestyle='--', alpha=0.5)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Roll Rate (rad/s)')
    axes[0, 0].set_title('Gyro X (Should be ~0)')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(df['t_s'], df['imu_gy_rps'], 'r-', linewidth=2)
    axes[0, 1].axhline(y=0, color='b', linestyle='--', alpha=0.5)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Pitch Rate (rad/s)')
    axes[0, 1].set_title('Gyro Y (Should be ~0)')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].plot(df['t_s'], yaw_rate_truth_rps, 'b-', label='Truth', linewidth=2)
    axes[1, 0].plot(df['t_s'], df['imu_gz_rps'], 'r--', label='Measured', alpha=0.7)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Yaw Rate (rad/s)')
    axes[1, 0].set_title('Gyro Z (Yaw Rate)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(df['t_s'], accel_x_truth, 'b-', label='Truth', linewidth=2)
    axes[1, 1].plot(df['t_s'], df['imu_ax_mps2'], 'r--', label='Measured', alpha=0.7)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Accel X (m/s²)')
    axes[1, 1].set_title('Accel X (Longitudinal)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    axes[2, 0].plot(df['t_s'], df['imu_ay_mps2'], 'r-', linewidth=2)
    axes[2, 0].axhline(y=0, color='b', linestyle='--', alpha=0.5)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Accel Y (m/s²)')
    axes[2, 0].set_title('Accel Y (Lateral)')
    axes[2, 0].grid(True, alpha=0.3)
    
    axes[2, 1].plot(df['t_s'], df['imu_az_mps2'], 'r-', linewidth=2)
    axes[2, 1].axhline(y=-9.81, color='b', linestyle='--', alpha=0.5)
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Accel Z (m/s²)')
    axes[2, 1].set_title('Accel Z (Should be ~-9.81)')
    axes[2, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/imu_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/imu_sensor.png")
    
    # ========================================
    # Figure 7: GNSS Sensor (Simple Validation)
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('GNSS Sensor (GPS) - Under Development', fontsize=16, fontweight='bold')
    
    axes[0, 0].plot(df['gnss_lon_deg'], df['gnss_lat_deg'], 'r-', linewidth=2)
    axes[0, 0].set_xlabel('Longitude (deg)')
    axes[0, 0].set_ylabel('Latitude (deg)')
    axes[0, 0].set_title('GNSS Trajectory (WGS84)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    axes[0, 1].plot(df['t_s'], df['gnss_alt_m'], 'g-', linewidth=2)
    axes[0, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Altitude (m MSL)')
    axes[0, 1].set_title('GNSS Altitude')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].plot(df['t_s'], df['gnss_vn_mps'], 'b-', label='North', linewidth=2)
    axes[1, 0].plot(df['t_s'], df['gnss_ve_mps'], 'r-', label='East', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_title('GNSS Velocity (NED Frame)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(df['t_s'], df['gnss_fix_type'], 'g-', linewidth=2, label='Fix Type')
    axes[1, 1].plot(df['t_s'], df['gnss_sat_count'], 'b--', linewidth=2, label='Sat Count')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Value')
    axes[1, 1].set_title('GNSS Quality')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/gnss_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/gnss_sensor.png")
    
    # ========================================
    # Figure 8: Radar Sensor (Simple Validation)
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Radar Sensor - Under Development', fontsize=16, fontweight='bold')
    
    axes[0, 0].plot(df['t_s'], df['radar_target_range_m'], 'b-', linewidth=2)
    axes[0, 0].axhline(y=50.0, color='k', linestyle='--', alpha=0.3)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Range (m)')
    axes[0, 0].set_title('Radar Range to Target')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(df['t_s'], df['radar_target_rel_vel_mps'], 'r-', linewidth=2)
    axes[0, 1].plot(df['t_s'], df['v_mps'], 'k--', alpha=0.3, label='Expected')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Range Rate (m/s)')
    axes[0, 1].set_title('Radar Doppler')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].plot(df['t_s'], df['radar_target_angle_deg'], 'g-', linewidth=2)
    axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Angle (deg)')
    axes[1, 0].set_title('Radar Angle')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(df['t_s'], df['radar_status'], 'k-', linewidth=2)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Status')
    axes[1, 1].set_title('Radar Status')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/radar_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/radar_sensor.png")
    
    # ========================================
    # Print Statistics
    # ========================================
    print("\n" + "="*60)
    print("SENSOR ERROR STATISTICS")
    print("="*60)
    
    print("\nBATTERY SENSOR (High Accuracy):")
    print(f"  SOC RMSE:     {soc_rmse:.3f}%")
    print(f"  Voltage RMSE: {v_rmse:.3f}V")
    print(f"  Current RMSE: {i_rmse:.3f}A")
    
    print("\nWHEEL SPEED SENSOR (High Accuracy):")
    wheel_rmse = np.sqrt(np.mean((df['wheel_fl_rps_meas'] - df['wheel_fl_rps_truth'])**2))
    print(f"  FL RMSE:      {wheel_rmse:.3f} rps")
    
    print("\nIMU SENSOR (Under Development):")
    print(f"  Gyro Z RMSE:  {np.sqrt(np.mean((df['imu_gz_rps'] - yaw_rate_truth_rps)**2)):.4f} rad/s")
    print(f"  Accel X RMSE: {np.sqrt(np.mean((df['imu_ax_mps2'] - accel_x_truth)**2)):.3f} m/s²")
    print(f"  Accel Z mean: {df['imu_az_mps2'].mean():.3f} m/s² (expected ~-9.81)")
    
    print("\nGNSS SENSOR (Under Development):")
    print(f"  Lat range:    {df['gnss_lat_deg'].min():.6f} to {df['gnss_lat_deg'].max():.6f} deg")
    print(f"  Lon range:    {df['gnss_lon_deg'].min():.6f} to {df['gnss_lon_deg'].max():.6f} deg")
    print(f"  Avg sats:     {df['gnss_sat_count'].mean():.1f}")
    
    print("\nRADAR SENSOR (Under Development):")
    print(f"  Range mean:   {df['radar_target_range_m'].mean():.2f}m (expected ~50m)")
    print(f"  Valid frames: {(df['radar_status'] == 1).sum()}/{len(df)} ({100*(df['radar_status']==1).sum()/len(df):.1f}%)")
    
    print("\n" + "="*60)
    print("✓ Complete sensor analysis finished!")
    print("✓ Detailed analysis: Battery & Wheels (most accurate)")
    print("✓ Basic validation: IMU/GNSS/Radar (still under development)")
    print("="*60)

if __name__ == "__main__":
    csv_file = sys.argv[1] if len(sys.argv) > 1 else "sim_out.csv"
    analyze_complete_sensors(csv_file)