// src/plant/plant_model.hpp
#pragma once

#include "plant/plant_main/plant_state.hpp"
#include "plant/subsystem_manager/subsystem_manager.hpp"
#include "plant/steer_subsystem/steer_plant.hpp"
#include "plant/drive_subsystem/drive_plant.hpp"
#include "plant/battery_subsystem/battery_plant.hpp"
#ifdef __ZEPHYR__
// On Zephyr subsystems are stored by value — include concrete types here
#  include "plant/steer_subsystem/steer_subsystem.hpp"
#  include "plant/drive_subsystem/drive_subsystem.hpp"
#  include "plant/wheel_subsystem/wheel_subsystem.hpp"
#  include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
#  include "plant/battery_subsystem/battery_subsystem.hpp"
#endif

namespace sim { struct ActuatorCmd; }

namespace plant {

// NEW: Geometry parameters for load transfer and 3-DOF dynamics
struct VehicleGeometry {
    double cg_height_m = 0.5;      // CoG height above ground [m]
    double cg_to_front_m = 1.4;    // Distance from CoG to front axle [m]
    double cg_to_rear_m = 1.4;     // Distance from CoG to rear axle [m] (should match L - cg_to_front)
    double yaw_inertia_kgm2 = 5000.0; // Yaw moment of inertia Iz [kg·m²]
};

// NEW: Dynamic model configuration
struct DynamicModelConfig {
    bool enabled = false;          // Enable Dugoff tire model (vs kinematic)
    double surface_mu = 0.72;      // Surface friction coefficient
};

struct PlantModelParams {
    SteerParams steer{};
    DriveParams drive{};
    BatteryPlantParams battery_params{};
    MotorParams motor_params{};

    double wheelbase_m = 2.8;
    double track_width_m = 1.6;
    
    // NEW: Geometry and dynamics
    VehicleGeometry geometry{};
    DynamicModelConfig dynamic_config{};
};

/**
 * PlantModel - Main vehicle physics orchestrator
 *
 * Subsystem execution order (priority-based):
 *   50:  SteerSubsystem     → δFL, δFR (Ackermann)
 *   100: DriveSubsystem     → τdrive_*, τbrake_*
 *   105: WheelSubsystem     → ω, Fx, Fy, Fz, σ, λ (Dugoff)
 *   110: VehicleSubsystem   → v, ψ, x, y
 *   150: BatterySubsystem   → SOC, V, I
 */
class PlantModel {
public:
    explicit PlantModel(PlantModelParams p = {});

    const PlantModelParams& params() const { return p_; }
    void set_params(const PlantModelParams& p);

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    SubsystemManager& subsystem_manager() { return subsystem_mgr_; }
    const SubsystemManager& subsystem_manager() const { return subsystem_mgr_; }

private:
    PlantModelParams p_;
    SubsystemManager subsystem_mgr_;

#ifdef __ZEPHYR__
    // Subsystems stored by value — no heap allocation.
    // Constructed and registered in plant_model_zephyr.cpp.
    SteerSubsystem   steer_;
    DriveSubsystem   drive_;
    WheelSubsystem   wheel_;
    VehicleSubsystem vehicle_;
    BatterySubsystem battery_;
#endif
};

} // namespace plant