#!/usr/bin/env python3
"""
Complete Sensor Suite Analysis - UPDATED FOR NEW FIELD NAMES
Analyzes all sensors: Battery, Wheels, IMU, GNSS, Radar
Matches CAN map specification field names
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def analyze_complete_sensors(csv_file):
    """Analyze complete sensor suite with truth comparison"""
    
    # Load data
    print(f"Loading {csv_file}...")
    df = pd.read_csv(csv_file)
    print(f"Loaded {len(df)} timesteps ({df['t_s'].iloc[-1]:.1f} seconds)\n")
    
    # Create output directory
    os.makedirs('plots', exist_ok=True)
    
    # ========================================
    # Figure 1: Battery & Wheel Sensors
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Battery & Wheel Speed Sensors', fontsize=16, fontweight='bold')
    
    # Battery SOC
    ax = axes[0, 0]
    ax.plot(df['t_s'], df['batt_soc_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['batt_soc_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('SOC (%)')
    ax.set_title('Battery State of Charge')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Battery Voltage
    ax = axes[0, 1]
    ax.plot(df['t_s'], df['batt_v_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['batt_v_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Voltage (V)')
    ax.set_title('Battery Voltage')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Battery Current
    ax = axes[1, 0]
    ax.plot(df['t_s'], df['batt_i_truth'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['batt_i_meas'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Current (A)')
    ax.set_title('Battery Current')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Wheel Speed (FL only for clarity)
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
    # Figure 2: IMU Sensor (6-DOF)
    # ========================================
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('IMU Sensor (6-DOF Gyroscope & Accelerometer)', fontsize=16, fontweight='bold')
    
    # Gyro X (Roll rate - should be ~0 for ground vehicle)
    ax = axes[0, 0]
    ax.plot(df['t_s'], df['imu_gx_rps'], 'r-', label='Measured', linewidth=2)
    ax.axhline(y=0, color='b', linestyle='--', label='Expected (0)', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Roll Rate (rad/s)')
    ax.set_title('Gyroscope X (Roll) - Should be ~0')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Gyro Y (Pitch rate - should be ~0 for ground vehicle)
    ax = axes[0, 1]
    ax.plot(df['t_s'], df['imu_gy_rps'], 'r-', label='Measured', linewidth=2)
    ax.axhline(y=0, color='b', linestyle='--', label='Expected (0)', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch Rate (rad/s)')
    ax.set_title('Gyroscope Y (Pitch) - Should be ~0')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Gyro Z (Yaw rate)
    ax = axes[1, 0]
    # Calculate truth yaw rate from yaw angle derivative (convert to rad/s)
    yaw_rate_truth_rps = np.gradient(df['yaw_deg'].values, df['t_s'].values)
    ax.plot(df['t_s'], yaw_rate_truth_rps, 'b-', label='Truth (from yaw)', linewidth=2)
    ax.plot(df['t_s'], df['imu_gz_rps'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw Rate (rad/s)')
    ax.set_title('Gyroscope Z (Yaw Rate)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Accel X (longitudinal)
    ax = axes[1, 1]
    # Calculate truth accel from velocity derivative
    accel_x_truth = np.gradient(df['v_mps'].values, df['t_s'].values)
    ax.plot(df['t_s'], accel_x_truth, 'b-', label='Truth (from velocity)', linewidth=2)
    ax.plot(df['t_s'], df['imu_ax_mps2'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Accel X (m/s²)')
    ax.set_title('Accelerometer X (Longitudinal)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Accel Y (lateral - from turning)
    ax = axes[2, 0]
    ax.plot(df['t_s'], df['imu_ay_mps2'], 'r-', label='Measured', linewidth=2)
    ax.axhline(y=0, color='b', linestyle='--', label='Expected (~0, no turning)', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Accel Y (m/s²)')
    ax.set_title('Accelerometer Y (Lateral)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Accel Z (vertical - should be ~-9.81 m/s² gravity)
    ax = axes[2, 1]
    ax.plot(df['t_s'], df['imu_az_mps2'], 'r-', label='Measured', linewidth=2)
    ax.axhline(y=-9.81, color='b', linestyle='--', label='Expected (-9.81)', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Accel Z (m/s²)')
    ax.set_title('Accelerometer Z (Gravity + Noise)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/imu_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/imu_sensor.png")
    
    # ========================================
    # Figure 3: GNSS Sensor (WGS84 + NED Velocity)
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('GNSS Sensor (GPS - WGS84 Position + NED Velocity)', fontsize=16, fontweight='bold')
    
    # Position trajectory (lat/lon in degrees)
    ax = axes[0, 0]
    ax.plot(df['gnss_lon_deg'], df['gnss_lat_deg'], 'r-', label='GNSS', linewidth=2)
    ax.set_xlabel('Longitude (deg)')
    ax.set_ylabel('Latitude (deg)')
    ax.set_title('GNSS Trajectory (WGS84)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Altitude
    ax = axes[0, 1]
    ax.plot(df['t_s'], df['gnss_alt_m'], 'g-', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='MSL (0m)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m MSL)')
    ax.set_title('GNSS Altitude')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Velocity North/East
    ax = axes[1, 0]
    ax.plot(df['t_s'], df['gnss_vn_mps'], 'b-', label='North', linewidth=2)
    ax.plot(df['t_s'], df['gnss_ve_mps'], 'r-', label='East', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('GNSS Velocity (NED Frame)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Fix quality
    ax = axes[1, 1]
    ax.plot(df['t_s'], df['gnss_fix_type'], 'g-', linewidth=2, label='Fix Type')
    ax.plot(df['t_s'], df['gnss_sat_count'], 'b--', linewidth=2, label='Sat Count')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    ax.set_title('GNSS Quality (Fix Type: 0=none, 3=3D)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/gnss_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/gnss_sensor.png")
    
    # ========================================
    # Figure 4: Radar Sensor
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Radar Sensor (Range & Doppler)', fontsize=16, fontweight='bold')
    
    # Range
    ax = axes[0, 0]
    ax.plot(df['t_s'], df['radar_target_range_m'], 'b-', linewidth=2)
    ax.axhline(y=50.0, color='k', linestyle='--', alpha=0.3, label='Expected (50m)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Range (m)')
    ax.set_title('Radar Range to Target')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Range rate (closing velocity)
    ax = axes[0, 1]
    ax.plot(df['t_s'], df['radar_target_rel_vel_mps'], 'r-', linewidth=2)
    # Expected: vehicle velocity (approaching stationary target)
    ax.plot(df['t_s'], df['v_mps'], 'k--', alpha=0.3, label='Expected (v_ego)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Range Rate (m/s)')
    ax.set_title('Radar Doppler (Closing Velocity)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Angle
    ax = axes[1, 0]
    ax.plot(df['t_s'], df['radar_target_angle_deg'], 'g-', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='Expected (0° ahead)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Radar Angle to Target')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Target status
    ax = axes[1, 1]
    ax.plot(df['t_s'], df['radar_status'], 'k-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Status Flags')
    ax.set_title('Radar Status (bit 0 = valid target)')
    ax.set_ylim([-0.1, 2])
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/radar_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/radar_sensor.png")
    
    # ========================================
    # Print Statistics
    # ========================================
    print("\n" + "="*60)
    print("SENSOR ERROR STATISTICS")
    print("="*60)
    
    # Battery
    print("\nBATTERY SENSOR:")
    print(f"  SOC RMSE:     {np.sqrt(np.mean((df['batt_soc_meas'] - df['batt_soc_truth'])**2)):.3f}%")
    print(f"  Voltage RMSE: {np.sqrt(np.mean((df['batt_v_meas'] - df['batt_v_truth'])**2)):.3f}V")
    print(f"  Current RMSE: {np.sqrt(np.mean((df['batt_i_meas'] - df['batt_i_truth'])**2)):.3f}A")
    
    # Wheels
    print("\nWHEEL SPEED SENSOR:")
    wheel_rmse = np.sqrt(np.mean((df['wheel_fl_rps_meas'] - df['wheel_fl_rps_truth'])**2))
    print(f"  FL RMSE:      {wheel_rmse:.3f} rps")
    
    # IMU
    print("\nIMU SENSOR:")
    print(f"  Gyro Z RMSE:  {np.sqrt(np.mean((df['imu_gz_rps'] - yaw_rate_truth_rps)**2)):.4f} rad/s")
    print(f"  Accel X RMSE: {np.sqrt(np.mean((df['imu_ax_mps2'] - accel_x_truth)**2)):.3f} m/s²")
    print(f"  Accel Z mean: {df['imu_az_mps2'].mean():.3f} m/s² (expected ~-9.81)")
    
    # GNSS
    print("\nGNSS SENSOR:")
    print(f"  Lat range:    {df['gnss_lat_deg'].min():.6f} to {df['gnss_lat_deg'].max():.6f} deg")
    print(f"  Lon range:    {df['gnss_lon_deg'].min():.6f} to {df['gnss_lon_deg'].max():.6f} deg")
    print(f"  Alt mean:     {df['gnss_alt_m'].mean():.2f}m MSL")
    print(f"  Velocity N:   {df['gnss_vn_mps'].mean():.2f} ± {df['gnss_vn_mps'].std():.2f} m/s")
    print(f"  Velocity E:   {df['gnss_ve_mps'].mean():.2f} ± {df['gnss_ve_mps'].std():.2f} m/s")
    print(f"  Avg sats:     {df['gnss_sat_count'].mean():.1f}")
    
    # Radar
    print("\nRADAR SENSOR:")
    print(f"  Range mean:   {df['radar_target_range_m'].mean():.2f}m (expected ~50m)")
    print(f"  Range RMSE:   {np.sqrt(np.mean((df['radar_target_range_m'] - 50.0)**2)):.3f}m")
    print(f"  Angle mean:   {df['radar_target_angle_deg'].mean():.2f}° (expected ~0°)")
    print(f"  Valid frames: {(df['radar_status'] == 1).sum()}/{len(df)} ({100*(df['radar_status']==1).sum()/len(df):.1f}%)")
    
    print("\n" + "="*60)
    print("✓ Complete sensor analysis finished!")
    print("="*60)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "sim_out.csv"
    
    analyze_complete_sensors(csv_file)