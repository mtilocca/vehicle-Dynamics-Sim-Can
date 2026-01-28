// src/plant/plant_main/vehicle_bicycle_ackermann.hpp
//
// VehicleBicycleAckermann - Vehicle position and velocity integration
//
// In the hybrid model architecture:
//   - DriveSubsystem: Computes tau_drive_*, tau_brake_*
//   - WheelSubsystem: Integrates wheel dynamics, computes Fx_*, Fy_*
//   - VehicleBicycleAckermann: Integrates position and velocity from ΣFx
//
// DYNAMIC MODE (dynamic_model_enabled=true):
//   - Position integration (x, y, yaw)
//   - Velocity integration from ΣFx (friction-limited)
//   - Enables realistic traction limiting, slip effects
//
// Reference: Vehicle_Dynamics_with_Dugoff_Tire_Model.pdf (Section 6, Eq. 4)

#pragma once

#include <cmath>
#include "plant/battery_subsystem/battery_plant.hpp"

namespace plant {

// Forward declaration
struct PlantState;

// ============================================================================
// BicycleAckermannParams - Configuration for vehicle model
// ============================================================================

struct BicycleAckermannParams {
    // ========================================================================
    // Geometry (XCMG XDE320 Table 4)
    // ========================================================================
    double L_m = 6.3;               // Wheelbase [m]
    double W_m = 4.0;               // Track width [m]
    double delta_max_rad = 0.52;    // Max steering angle [rad] (~30°)

    // ========================================================================
    // Lateral Dynamics
    // ========================================================================
    double mu_lat = 0.72;           // Lateral friction coefficient
    double g = 9.81;                // Gravity [m/s²]
    
    // ========================================================================
    // Longitudinal Dynamics (for DYNAMIC mode)
    // ========================================================================
    bool dynamic_model_enabled = false;  // false=kinematic, true=force-based
    
    double mass_kg = 218000.0;      // Vehicle mass [kg]
    double drag_c = 2.5;            // Aerodynamic drag [N/(m/s)²]
    double roll_c = 1500.0;         // Rolling resistance [N]
    
    // Traction limiting (only used when dynamic_model_enabled=true)
    bool traction_limiting = true;
};

// ============================================================================
// BicycleState2D - Vehicle pose
// ============================================================================

struct BicycleState2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
    double speed_mps = 0.0;
};

// ============================================================================
// BicycleStepResult - Output from step()
// ============================================================================

struct BicycleStepResult {
    BicycleState2D next;
    double yaw_rate_rps = 0.0;
    double delta_fl_rad = 0.0;
    double delta_fr_rad = 0.0;
    
    // Dynamic mode outputs
    double a_long_mps2 = 0.0;       // Longitudinal acceleration
    double Fx_total = 0.0;          // Total longitudinal tire force used
    double Fx_available = 0.0;      // Maximum available traction
};

// ============================================================================
// VehicleBicycleAckermann - Vehicle dynamics
// ============================================================================

class VehicleBicycleAckermann {
public:
    /**
     * ackermann_map() - Compute individual wheel angles from virtual steering
     * 
     * Uses Ackermann geometry to compute inner/outer wheel angles.
     */
    static void ackermann_map(
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double& delta_fl_rad,
        double& delta_fr_rad,
        double* curvature_out = nullptr
    );

    /**
     * step() - Update vehicle position and velocity
     * 
     * KINEMATIC MODE:
     *   - Position update: x += v·cos(ψ)·dt, y += v·sin(ψ)·dt, ψ += ψ̇·dt
     *   - Velocity is set externally by DriveSubsystem
     * 
     * DYNAMIC MODE (PDF Eq. 4):
     *   - m·dv/dt = ΣFx - F_drag - F_roll
     *   - Uses Fx_* from all 4 wheels (set by WheelSubsystem)
     *   - Enables friction-limited acceleration
     * 
     * @param s             Current pose
     * @param v_mps         Current velocity [m/s]
     * @param steer_virtual Virtual steering angle [rad]
     * @param p             Model parameters
     * @param dt_s          Timestep [s]
     * @param battery_plant Battery for energy tracking (legacy)
     * @param state         PlantState with tire forces (for dynamic mode)
     */
    static BicycleStepResult step(
        const BicycleState2D& s,
        double v_mps,
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double dt_s,
        BatteryPlant& battery_plant,
        const PlantState& state
    );
    
    /**
     * Legacy overload for backward compatibility (uses kinematic mode)
     */
    static BicycleStepResult step(
        const BicycleState2D& s,
        double v_mps,
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double dt_s,
        BatteryPlant& battery_plant
    );
};

} // namespace plant