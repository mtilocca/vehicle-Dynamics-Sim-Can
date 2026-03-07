// src/sim/controller_schedule.hpp
//
// Closed-loop controller configuration — gentle slalom at 5 m/s.
// Edit this file to change target speed, torque %, and steer timing.
//
// Vehicle params (from heavy_truck.yaml):
//   mass_kg              = 218,000 kg
//   motor_torque_max_nm  = 9,500 Nm (controller hardcodes this value)
//   brake_torque_max_nm  = 80,000 Nm
//   roll_c               = 15,000 N   (Crr ≈ 0.007)
//   drag_c               = 3.3 N/(m/s)²
//
// Steer sign convention (per steer_plant.cpp Ackermann model):
//   POSITIVE steer_cmd_deg  →  turn RIGHT  (forward driving)
//   NEGATIVE steer_cmd_deg  →  turn LEFT   (forward driving)
//
// Traction sign convention (drive_plant.cpp):
//   POSITIVE drive_torque_cmd_nm  →  forward traction
//
// Phase sequence (slalom only, ~90 s total):
//   WAIT_CAN   — no CAN data yet, hold zero
//   ACCEL      — forward torque until v >= 5 m/s               [speed-based exit, ~10 s]
//   CRUISE     — straight at 5 m/s                             [5 s]
//   STEER_L    — gentle left (-8°) at 5 m/s                   [10 s]
//   SETTLE1    — straight at 5 m/s                             [5 s]
//   STEER_R    — gentle right (+8°) at 5 m/s                  [10 s]
//   SETTLE2    — straight at 5 m/s                             [5 s]
//   STEER_L2   — gentle left (-8°) at 5 m/s  (gate 3)         [10 s]
//   SETTLE3    — straight at 5 m/s                             [5 s]
//   STEER_R2   — gentle right (+8°) at 5 m/s (gate 4)         [10 s]
//   SETTLE4    — straight at 5 m/s                             [5 s]
//   BRAKE_STOP — brake to full stop                            [speed-based, ~15 s]
//   STOPPED    — send zeros and exit

#pragma once

namespace controller {

// ============================================================================
// Speed control
// ============================================================================

/// Target longitudinal speed [m/s]  (5 m/s ≈ 18 km/h)
static constexpr double TARGET_SPEED_MPS = 5.0;

/// Torque fraction of MOTOR_TQ_MAX_NM (9500 Nm hardcoded in main).
/// 70% × 9500 = 6650 Nm — plenty to overcome roll_c=15000 N and accelerate.
static constexpr double TORQUE_FRACTION = 0.70;

/// Bang-bang deadband (±15% of target speed)
static constexpr double TOLERANCE = 0.15;   // ±15 %

/// Brake pedal % during deceleration [0–100 %]
/// 15% of 80 kNm → F_brake ≈ 6.2 kN → comfortable stop from 5 m/s
static constexpr double BRAKE_PCT_DECEL = 15.0;

// ============================================================================
// Slalom phase durations
// ============================================================================

/// Straight cruise before first gate [s]
static constexpr double CRUISE_DUR_S  = 5.0;

/// Duration of each steer gate (L or R) [s]
static constexpr double STEER_DUR_S   = 10.0;

/// Settle (straight) between gates [s]
static constexpr double SETTLE_DUR_S  = 5.0;

// ============================================================================
// Slalom steer angles — POSITIVE = RIGHT, NEGATIVE = LEFT
// ============================================================================

/// Gentle left steer for slalom gates 1 and 3 [deg]
static constexpr double STEER_LEFT_DEG  = -8.0;

/// Gentle right steer for slalom gates 2 and 4 [deg]
static constexpr double STEER_RIGHT_DEG = +8.0;

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
