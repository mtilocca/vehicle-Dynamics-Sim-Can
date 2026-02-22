// src/sim/controller_schedule.hpp
//
// Closed-loop controller configuration.
// Edit this file to change target speed, torque %, and steer timing.
//
// Vehicle params (from heavy_truck.yaml):
//   mass_kg              = 218,000 kg
//   motor_torque_max_nm  = 9,500 Nm (input shaft)
//   gear_ratio           = 25
//   wheel_radius_m       = 1.93 m
//   brake_torque_max_nm  = 80,000 Nm
//   Cy_front_Npm         = 2,500,000 N/rad (axle total)
//   Cy_rear_Npm          = 2,000,000 N/rad (axle total)
//   roll_c               = 15,000 N   (Crr ≈ 0.007)
//   drag_c               = 3.3 N/(m/s)²
//
// Steer sign convention (per steer_plant.cpp Ackermann model):
//   POSITIVE steer_cmd_deg  →  turn RIGHT  (forward driving)
//   NEGATIVE steer_cmd_deg  →  turn LEFT   (forward driving)
//   Direction of curvature reverses when driving in reverse.
//
// Traction sign convention (drive_plant.cpp — NOT gear-dependent):
//   POSITIVE drive_torque_cmd_nm  →  forward traction
//   NEGATIVE drive_torque_cmd_nm  →  reverse traction
//   gear_position=REVERSE clamps velocity ≤ 0 as a safety net.
//
// Full phase sequence:
//   WAIT_CAN    — no CAN data yet, hold zero
//   ACCEL       — full forward torque until v >= target*(1-TOL)
//   CRUISE      — hold speed straight for CRUISE_DUR_S seconds
//   STEER_L     — hold speed + STEER_LEFT_DEG for STEER_DUR_S seconds
//   SETTLE1     — steer=0, hold speed for SETTLE_DUR_S seconds
//   STEER_R     — hold speed + STEER_RIGHT_DEG for STEER_DUR_S seconds
//   SETTLE2     — steer=0, hold speed for SETTLE_DUR_S seconds
//   BRAKE_STOP  — gear=FORWARD, brake to full stop
//   REV_ACCEL   — gear=REVERSE, negative torque, accelerate to -TARGET_SPEED_REV_MPS
//   REV_STEER   — hold reverse speed + REV_STEER_DEG for REV_STEER_DUR_S seconds
//   REV_SETTLE  — steer=0, hold reverse speed for REV_SETTLE_DUR_S seconds
//   REV_BRAKE   — gear=REVERSE, brake to full stop (tests reverse brake clamping)
//   FWD2_ACCEL  — gear=FORWARD, torque until v >= 10 km/h
//   FWD2_COAST  — release everything, let drag+rolling slow vehicle to 0
//   STOPPED     — send zeros and exit

#pragma once

namespace controller {

// ============================================================================
// Forward speed control
// ============================================================================

/// Target longitudinal speed [m/s]  (~25 km/h)
static constexpr double TARGET_SPEED_MPS = 7.0;

/// Torque command as a fraction of motor_torque_max_nm when accelerating.
/// motor_torque_max_nm = 95,000 Nm (YAML) × gear_ratio=25 × η=0.92 / R=1.93
/// → peak F_traction = 1,132 kN.  At 7 %:  F = 79 kN → a_net ≈ 0.295 m/s²
/// → 0→25 km/h in ~24 s (realistic empty haul truck).
static constexpr double TORQUE_FRACTION = 0.7;

/// On-off deadband (both forward and reverse speed controllers)
static constexpr double TOLERANCE = 0.15;   // ±15 %

/// Brake pedal percentage applied during deceleration [0–100 %]
/// 40 % of 80 kNm max → F_brake ≈ 16.6 kN → a_brake ≈ 0.076 m/s²
/// (very heavy loaded feel, ~130 m stopping distance from 25 km/h)
static constexpr double BRAKE_PCT_DECEL = 10.0;

// ============================================================================
// Forward phase durations
// ============================================================================

/// Time at target speed, zero steer, before first steer event [s]
static constexpr double CRUISE_DUR_S = 10.0;

/// Duration of each forward steer event [s]
static constexpr double STEER_DUR_S = 20.0;

/// Settle time between steer events (steer=0, hold speed) [s]
static constexpr double SETTLE_DUR_S = 15.0;

// ============================================================================
// Forward steer angles — sign: POSITIVE = RIGHT, NEGATIVE = LEFT
// ============================================================================

static constexpr double STEER_LEFT_DEG  = -10.0;
static constexpr double STEER_RIGHT_DEG = +10.0;

// ============================================================================
// Reverse U-turn parameters
// ============================================================================

/// Target reverse speed magnitude [m/s]  (vehicle will reach v ≈ -TARGET_SPEED_REV_MPS)
static constexpr double TARGET_SPEED_REV_MPS = 3.0;   // ~11 km/h

/// Fraction of motor_torque_max_nm for reverse acceleration (gentler than forward).
/// Torque command will be NEGATIVE → drive_plant produces backward traction force.
/// NOTE: controller uses MOTOR_TQ_MAX_NM = 9500 Nm (hardcoded), NOT the YAML 95,000 Nm.
/// Minimum to overcome roll_c=15,000 N: tq > 1,258 Nm → fraction > 0.13
/// At 50 %: tq = 4,750 Nm → F_rev = 56.6 kN → a_net ≈ 0.19 m/s² → 0→11 km/h in ~16 s
static constexpr double TORQUE_FRACTION_REV = 0.50;

/// Steer angle during reverse U-turn [deg]  (+15° right steer for curve)
static constexpr double REV_STEER_DEG = +15.0;

/// Duration of the reverse steer event [s]
static constexpr double REV_STEER_DUR_S = 15.0;

/// Short straight reverse phase after steer, before applying reverse brake [s]
static constexpr double REV_SETTLE_DUR_S = 5.0;

// ============================================================================
// Resistive-forces coast test (second forward run)
// ============================================================================

/// Target speed for the second forward run [m/s]  (10 km/h)
static constexpr double TARGET_SPEED_FWD2_MPS = 10.0 / 3.6;   // ≈ 2.78 m/s

/// Torque fraction for the second forward run (same as primary)
static constexpr double TORQUE_FRACTION_FWD2 = 0.70;

/// Maximum time to coast before declaring the test done [s]
/// At roll_c=15000 N, 218 t: decel ≈ 0.069 m/s², coast from 2.78 m/s ≈ 40 s
static constexpr double FWD2_COAST_MAX_S = 90.0;

// ============================================================================
// CAN interface
// ============================================================================

static constexpr const char* CAN_INTERFACE  = "vcan0";
static constexpr const char* CAN_MAP_PATH   = "config/can_map.csv";
static constexpr const char* CMD_FRAME_NAME = "ACTUATOR_CMD_1";

/// Command TX rate [Hz] — matches ACTUATOR_CMD_1 nominal cycle of 10 ms
static constexpr double CMD_HZ = 100.0;

/// Safety: if no state frame received for this long, go to safe mode
static constexpr double CAN_TIMEOUT_S = 1.0;

} // namespace controller
