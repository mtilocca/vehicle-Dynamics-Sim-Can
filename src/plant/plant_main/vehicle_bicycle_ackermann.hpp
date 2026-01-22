// src/plant/plant_main/vehicle_bicycle_ackermann.hpp
#pragma once

#include <cmath>
#include "plant/battery_subsystem/battery_plant.hpp"  // Include BatteryPlant class for energy management

namespace plant {

// Forward declaration to avoid circular include
struct PlantState;

struct BicycleAckermannParams {
    // --- Existing geometry parameters ---
    double L_m = 2.8;           // Wheelbase
    double W_m = 1.6;           // Track width
    double delta_max_rad = 0.6; // Max steering angle

    // --- Existing realism knob ---
    double mu_lat = 0.9;        // Lateral friction coefficient
    double g = 9.81;            // Gravitational acceleration
    
    // ========================================================================
    // NEW: Dynamic Model Parameters (for force-based dynamics)
    // ========================================================================
    bool dynamic_model_enabled = false;  // false=kinematic, true=force-based
    
    // Vehicle dynamics (only used when dynamic_model_enabled=true)
    double mass_kg = 1800.0;    // Vehicle mass for F=ma
    double drag_c = 0.35;       // Aerodynamic drag coefficient (N/(m/s)²)
    double roll_c = 40.0;       // Rolling resistance (N)
    
    // Tire force scaling (for traction limiting)
    // When enabled, acceleration is limited by available tire forces
    bool traction_limiting = true;
};

struct BicycleState2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
    double speed_mps = 0.0;  // Add speed for energy calculation and dynamic mode
};

struct BicycleStepResult {
    BicycleState2D next;
    double yaw_rate_rps = 0.0;
    double delta_fl_rad = 0.0;
    double delta_fr_rad = 0.0;
    
    // --- NEW: Dynamic model outputs ---
    double a_long_mps2 = 0.0;   // Longitudinal acceleration (from tire forces)
    double Fx_total = 0.0;     // Total longitudinal tire force used
    double Fx_available = 0.0; // Available traction (friction limited)
};

class VehicleBicycleAckermann {
public:
    /**
     * ackermann_map() - Map virtual steering angle to individual wheel angles
     * 
     * Computes Ackermann geometry for inner/outer wheel angles.
     * This function is unchanged from V1.
     */
    static void ackermann_map(
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double& delta_fl_rad,
        double& delta_fr_rad,
        double* curvature_out = nullptr
    );

    /**
     * step() - Update vehicle position and orientation
     * 
     * KINEMATIC MODE (dynamic_model_enabled=false):
     *   - Pure kinematic bicycle model
     *   - Position update: x += v*cos(ψ)*dt, y += v*sin(ψ)*dt
     *   - Velocity is set externally by DriveSubsystem
     *   - No slip, unlimited traction
     * 
     * DYNAMIC MODE (dynamic_model_enabled=true):
     *   - Force-based acceleration from Dugoff tire model
     *   - a = (Fx_tire - F_drag - F_roll) / m
     *   - v_next = v + a*dt
     *   - Traction limited by available tire forces from PlantState
     * 
     * @param s Current pose (x, y, yaw)
     * @param v_mps Current longitudinal velocity
     * @param steer_virtual_rad Virtual (bicycle) steering angle
     * @param p Model parameters
     * @param dt_s Timestep
     * @param battery_plant Reference for energy tracking (legacy)
     * @param state PlantState with tire forces (for dynamic mode)
     * @return Updated pose and diagnostic outputs
     */
    static BicycleStepResult step(
        const BicycleState2D& s,
        double v_mps,
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double dt_s,
        BatteryPlant& battery_plant,
        const PlantState& state  // NEW: for tire forces in dynamic mode
    );
    
    // --- LEGACY OVERLOAD (backward compatibility) ---
    // Calls new step() with default PlantState
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