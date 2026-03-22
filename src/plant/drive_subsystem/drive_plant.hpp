// src/plant/drive_subsystem/drive_plant.hpp
//
// DrivePlant - Powertrain physics (motor torque, braking, energy flow)
//
// In the hybrid model architecture:
//   - DrivePlant computes motor/brake torques and populates tau_drive_*, tau_brake_*
//   - WheelSubsystem integrates wheel dynamics and computes tire forces
//   - VehicleBicycleAckermann integrates vehicle velocity from ΣFx
//
// This separation enables proper slip-based tire force computation.

#pragma once

#include <algorithm>
#include "plant/plant_main/plant_state.hpp"
#include "plant/battery_subsystem/battery_plant.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct DriveParams {
    // ========================================================================
    // Vehicle Mass (XCMG XDE320 Table 4)
    // ========================================================================
    double mass_kg = 218000.0;          // Empty mass [kg]
    // Loaded: 538000 kg (with 320t payload)
    
    // ========================================================================
    // Wheel/Tire Geometry (XCMG XDE320 Table 4)
    // ========================================================================
    double wheel_radius_m = 1.93;       // 37.00R57 effective radius [m]
    
    // ========================================================================
    // Resistive Forces
    // ========================================================================
    double drag_c = 2.5;                // Aerodynamic drag [N/(m/s)²]
    double roll_c = 1500.0;             // Rolling resistance [N] (μ_roll × m × g)
    
    // ========================================================================
    // Motor Limits (XCMG XDE320 Table 4)
    // ========================================================================
    double motor_torque_max_nm = 9500.0;   // Peak motor torque [Nm]
    double motor_power_max_w = 2800000.0;  // 2800 kW continuous [W]
    
    // ========================================================================
    // Brake System
    // ========================================================================
    double brake_torque_max_nm = 80000.0;  // Max brake torque per axle [Nm]
    double brake_bias_front = 0.40;        // Front axle brake proportion
    // Rear gets (1 - brake_bias_front) = 60%
    // Mining trucks are rear-heavy, need more rear braking
    
    // ========================================================================
    // Drivetrain (XCMG XDE320 - electric drive)
    // ========================================================================
    double gear_ratio = 25.0;           // Final drive ratio
    double drivetrain_eff = 0.92;       // Drivetrain efficiency
    
    // Note: XCMG XDE320 is rear-wheel drive (RWD)
    // Front wheels are non-driven, steering only
    
    // ========================================================================
    // Regenerative Braking
    // ========================================================================
    double regen_eff_active = 0.68;     // Efficiency during active regen
    double regen_eff_coast = 0.03;      // Efficiency during coasting
    
    // ========================================================================
    // Speed Limits
    // ========================================================================
    double v_stop_eps = 0.3;            // Standstill threshold [m/s]
    double v_max_mps = 60.0;            // Maximum velocity [m/s] (~216 km/h)
};

// ============================================================================
// DrivePlant - Powertrain physics
// ============================================================================

class DrivePlant {
public:
    explicit DrivePlant(DriveParams p = {}, BatteryPlant* battery_plant = nullptr)
        : p_(p), battery_plant_(battery_plant) {}

    /**
     * step() - Main powertrain update
     * 
     * Outputs to PlantState:
     * - tau_drive_rl_nm, tau_drive_rr_nm: Drive torques for rear wheels
     * - tau_brake_*_nm: Brake torques for all wheels (with bias)
     * - motor_torque_nm, motor_power_kW: Motor state
     * - batt_i, batt_v: Battery state (if connected)
     * 
     * In KINEMATIC mode only:
     * - v_mps, a_long_mps2: Velocity integration
     * - wheel_*_rps: Wheel speeds (derived from v)
     * 
     * In DYNAMIC mode:
     * - WheelSubsystem handles wheel speeds
     * - VehicleBicycleAckermann handles velocity
     */
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const DriveParams& params() const { return p_; }
    DriveParams& params() { return p_; }

    void set_battery_plant(BatteryPlant* battery) { battery_plant_ = battery; }

private:
    DriveParams p_;
    BatteryPlant* battery_plant_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }
};

} // namespace plant