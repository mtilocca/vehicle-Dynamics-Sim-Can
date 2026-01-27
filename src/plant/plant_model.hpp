// src/plant/plant_model.hpp
#pragma once

#include "plant/plant_main/plant_state.hpp"
#include "plant/subsystem_manager/subsystem_manager.hpp"
#include "plant/steer_subsystem/steer_plant.hpp"
#include "plant/drive_subsystem/drive_plant.hpp"
#include "plant/battery_subsystem/battery_plant.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

// ============================================================================
// Vehicle Geometry Parameters (NEW - for dynamic model / tire subsystem)
// ============================================================================
// These parameters define the vehicle's physical geometry needed for:
// - Normal load distribution (CoG position)
// - Load transfer during acceleration/braking
// - Tire force calculations
struct VehicleGeometry {
    // --- Basic dimensions ---
    double wheelbase_m = 2.8;       // Distance between front and rear axles
    double track_width_m = 1.6;     // Distance between left and right wheels
    double wheel_radius_m = 0.33;   // Effective tire radius
    
    // --- Center of Gravity (CoG) position ---
    double cg_height_m = 0.5;       // CoG height above ground
    double cg_to_front_m = 1.4;     // Distance from CoG to front axle
    double cg_to_rear_m = 1.4;      // Distance from CoG to rear axle
    // Note: cg_to_front + cg_to_rear should equal wheelbase_m
    
    // --- Derived helpers ---
    double front_weight_ratio() const {
        return cg_to_rear_m / wheelbase_m;
    }
    
    double rear_weight_ratio() const {
        return cg_to_front_m / wheelbase_m;
    }
};

// ============================================================================
// Dynamic Model Configuration 
// ============================================================================
// Controls whether to use kinematic or force-based dynamics
struct DynamicModelConfig {
    bool enabled = false;           // false=kinematic, true=dynamic (Dugoff) // REMOVE IT AS KINEMATIC WILL BE GONE 
    double surface_mu = 0.72;       // Default surface friction coefficient
    bool traction_limiting = true;  // Enable tire traction limiting
};

struct PlantModelParams {
    SteerParams steer{};
    DriveParams drive{};
    BatteryPlantParams battery_params{};
    MotorParams motor_params{};

    double wheelbase_m = 2.8;
    double track_width_m = 1.6;
    
    // --- NEW: Vehicle geometry for dynamic model ---
    VehicleGeometry geometry{};
    
    // --- NEW: Dynamic model configuration ---
    DynamicModelConfig dynamic_config{};
};

/**
 * PlantModel - Main vehicle physics orchestrator
 * 
 * Now uses SubsystemManager for scalable subsystem architecture.
 * Subsystems are registered by priority and executed automatically.
 * 
 * Execution order (by priority):
 *   50  - Steer    (steering angles, Ackermann mapping)
 *   100 - Drive    (motor force demand, velocity integration in kinematic mode)
 *   105 - Tyre     (normal loads, slip ratios, Dugoff forces) [NEW]
 *   150 - Battery  (energy tracking, SOC update)
 * 
 * After subsystems, VehicleBicycleAckermann updates position/yaw:
 *   - Kinematic mode: position from v*dt
 *   - Dynamic mode: acceleration from tire forces, then position from v*dt
 */
class PlantModel {
public:
    explicit PlantModel(PlantModelParams p = {});

    const PlantModelParams& params() const { return p_; }
    void set_params(const PlantModelParams& p);

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    // Access to subsystem manager (for advanced control)
    SubsystemManager& subsystem_manager() { return subsystem_mgr_; }
    const SubsystemManager& subsystem_manager() const { return subsystem_mgr_; }
    
    // --- NEW: Dynamic model control ---
    /**
     * set_dynamic_model_enabled() - Switch between kinematic and dynamic modes
     * 
     * When enabled:
     *   - TyreSubsystem computes Dugoff tire forces
     *   - DriveSubsystem uses traction-limited forces
     *   - VehicleBicycleAckermann uses force-based acceleration
     * 
     * When disabled (default):
     *   - Pure kinematic bicycle model (V1 behavior)
     *   - Unlimited traction (no slip)
     */
    void set_dynamic_model_enabled(bool enabled);
    bool is_dynamic_model_enabled() const { return p_.dynamic_config.enabled; }
    
    /**
     * set_surface_friction() - Set current road surface friction coefficient
     * 
     * Common values (from Pilbara mining conditions):
     *   - Dry pavement: 0.85
     *   - Gravel compact: 0.72 (baseline)
     *   - Gravel loose: 0.55
     *   - Iron ore dust (dry): 0.45
     *   - Iron ore dust (wet): 0.30
     *   - Mud: 0.25
     */
    void set_surface_friction(double mu);
    double get_surface_friction() const { return p_.dynamic_config.surface_mu; }

private:
    PlantModelParams p_;
    SubsystemManager subsystem_mgr_;
};

} // namespace plant