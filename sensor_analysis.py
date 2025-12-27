#!/usr/bin/env python3
"""
Complete Sensor Suite Analysis
Analyzes all sensors: Battery, Wheels, IMU, GNSS, Radar
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
    # Figure 2: IMU Sensor
    # ========================================
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('IMU Sensor (Gyroscope & Accelerometer)', fontsize=16, fontweight='bold')
    
    # Gyro - Yaw Rate
    ax = axes[0]
    # Calculate truth yaw rate from yaw angle derivative
    yaw_rate_truth = np.gradient(df['yaw_deg'].values, df['t_s'].values)
    ax.plot(df['t_s'], yaw_rate_truth, 'b-', label='Truth (from yaw)', linewidth=2)
    ax.plot(df['t_s'], df['imu_gyro_yaw_dps'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw Rate (deg/s)')
    ax.set_title('Gyroscope - Yaw Rate')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Accel X (longitudinal)
    ax = axes[1]
    # Calculate truth accel from velocity derivative
    accel_x_truth = np.gradient(df['v_mps'].values, df['t_s'].values)
    ax.plot(df['t_s'], accel_x_truth, 'b-', label='Truth (from velocity)', linewidth=2)
    ax.plot(df['t_s'], df['imu_accel_x_mps2'], 'r--', label='Measured', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Accel X (m/s²)')
    ax.set_title('Accelerometer - Longitudinal')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Accel Y (lateral)
    ax = axes[2]
    ax.plot(df['t_s'], df['imu_accel_y_mps2'], 'r-', label='Measured (lateral)', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Accel Y (m/s²)')
    ax.set_title('Accelerometer - Lateral (from turning)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/imu_sensor.png', dpi=150, bbox_inches='tight')
    print("✓ Saved: plots/imu_sensor.png")
    
    # ========================================
    # Figure 3: GNSS Sensor
    # ========================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('GNSS Sensor (GPS)', fontsize=16, fontweight='bold')
    
    # Position trajectory
    ax = axes[0, 0]
    ax.plot(df['x_m'], df['y_m'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['gnss_pos_x_m'], df['gnss_pos_y_m'], 'r--', label='GNSS', alpha=0.7)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Vehicle Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Position X error
    ax = axes[0, 1]
    pos_x_error = df['gnss_pos_x_m'] - df['x_m']
    ax.plot(df['t_s'], pos_x_error, 'g-', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X Position Error (m)')
    ax.set_title(f'GNSS X Error (RMSE: {np.sqrt(np.mean(pos_x_error**2)):.2f}m)')
    ax.grid(True, alpha=0.3)
    
    # Velocity
    ax = axes[1, 0]
    ax.plot(df['t_s'], df['v_mps'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['gnss_vel_mps'], 'r--', label='GNSS', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('GNSS Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Heading
    ax = axes[1, 1]
    ax.plot(df['t_s'], df['yaw_deg'], 'b-', label='Truth', linewidth=2)
    ax.plot(df['t_s'], df['gnss_hdg_deg'], 'r--', label='GNSS', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading (deg)')
    ax.set_title('GNSS Heading')
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
    ax.plot(df['t_s'], df['radar_range_m'], 'b-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Range (m)')
    ax.set_title('Radar Range to Target')
    ax.grid(True, alpha=0.3)
    
    # Range rate (closing velocity)
    ax = axes[0, 1]
    ax.plot(df['t_s'], df['radar_rate_mps'], 'r-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Range Rate (m/s)')
    ax.set_title('Radar Doppler (Closing Velocity)')
    ax.grid(True, alpha=0.3)
    
    # Angle
    ax = axes[1, 0]
    ax.plot(df['t_s'], df['radar_angle_deg'], 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Radar Angle to Target')
    ax.grid(True, alpha=0.3)
    
    # Target validity
    ax = axes[1, 1]
    ax.plot(df['t_s'], df['radar_valid'], 'k-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Valid Target')
    ax.set_title('Radar Target Detection')
    ax.set_ylim([-0.1, 1.1])
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
    
    # GNSS
    print("\nGNSS SENSOR:")
    pos_x_rmse = np.sqrt(np.mean((df['gnss_pos_x_m'] - df['x_m'])**2))
    pos_y_rmse = np.sqrt(np.mean((df['gnss_pos_y_m'] - df['y_m'])**2))
    print(f"  Position X RMSE: {pos_x_rmse:.2f}m")
    print(f"  Position Y RMSE: {pos_y_rmse:.2f}m")
    print(f"  2D Position RMSE: {np.sqrt(pos_x_rmse**2 + pos_y_rmse**2):.2f}m")
    print(f"  Velocity RMSE: {np.sqrt(np.mean((df['gnss_vel_mps'] - df['v_mps'])**2)):.3f} m/s")
    
    print("\n" + "="*60)
    print("✓ Complete sensor analysis finished!")
    print("="*60)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "sim_out.csv"
    
    analyze_complete_sensors(csv_file)