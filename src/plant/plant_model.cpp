// src/plant/plant_model.cpp
#include "plant/plant_model.hpp"
#include "plant/plant_main/vehicle_bicycle_ackermann.hpp"
#include "plant/battery_subsystem/battery_subsystem.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/tyre_subsystem/tyre_subsystem.hpp"  // TODO --> replace wheel_subsystem and set new priority to 106
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"

namespace plant {

PlantModel::PlantModel(PlantModelParams p)
    : p_(p),
      subsystem_mgr_()
{
    // Initialize geometry parameters
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    
    // Sync geometry struct with legacy params
    p_.geometry.wheelbase_m = p_.wheelbase_m;
    p_.geometry.track_width_m = p_.track_width_m;
    p_.geometry.wheel_radius_m = p_.drive.wheel_radius_m;
    
    LOG_INFO("[PlantModel] Initializing with SubsystemManager architecture");
    LOG_INFO("[PlantModel] Battery: %.1f kWh, Motor: %.1f kW, Mass: %.0f kg",
             p_.battery_params.capacity_kWh, p_.motor_params.max_power_kW, p_.drive.mass_kg);
    LOG_INFO("[PlantModel] Geometry: wheelbase=%.2f m, track=%.2f m, CoG_h=%.2f m",
             p_.geometry.wheelbase_m, p_.geometry.track_width_m, p_.geometry.cg_height_m);

    // ========================================================================
    // Register Subsystems (auto-sorted by priority)
    // ========================================================================
    
    // 1. SteerSubsystem (Priority 50 - Actuators)
    auto steer = std::make_unique<SteerSubsystem>(p_.steer);
    subsystem_mgr_.register_subsystem(std::move(steer));
    
    // 2. BatterySubsystem (Priority 150 - Energy)
    auto battery = std::make_unique<BatterySubsystem>(
        p_.battery_params,
        p_.motor_params
    );
    auto* battery_ptr = battery.get();  // Keep pointer for Drive dependency injection
    subsystem_mgr_.register_subsystem(std::move(battery));
    
    // 3. DriveSubsystem (Priority 100 - Dynamics)
    auto drive = std::make_unique<DriveSubsystem>(p_.drive);
    drive->set_battery_subsystem(battery_ptr);  // Inject battery dependency
    subsystem_mgr_.register_subsystem(std::move(drive));
    
    // ========================================================================
    // 4. TyreSubsystem (Priority 105 - Tire Forces) --> REPLACE WITH WHEEL_SUBSYSTEM
    // ========================================================================
    // Register with vehicle geometry parameters
    auto tyre = std::make_unique<TyreSubsystem>(
        p_.geometry.wheelbase_m,
        p_.geometry.track_width_m,
        p_.drive.mass_kg,
        p_.geometry.cg_height_m,
        p_.geometry.cg_to_front_m,
        p_.geometry.cg_to_rear_m
    );
    
    // Set tire radius from drive params
    tyre->set_tire_radius(p_.drive.wheel_radius_m);
    
    // Start disabled for backward compatibility (kinematic mode)
    tyre->set_dynamic_model_enabled(p_.dynamic_config.enabled);
    
    subsystem_mgr_.register_subsystem(std::move(tyre));
    
    // Initialize all subsystems with default state
    PlantState init_state{};
    init_state.batt_soc_pct = 50.0;
    init_state.dynamic_model_enabled = p_.dynamic_config.enabled;
    init_state.surface_mu = p_.dynamic_config.surface_mu;
    subsystem_mgr_.initialize_all(init_state);
    
    LOG_INFO("[PlantModel] Registered %zu subsystems", subsystem_mgr_.subsystem_count());
    LOG_INFO("[PlantModel] Dynamic model: %s", p_.dynamic_config.enabled ? "ENABLED (Dugoff)" : "DISABLED (Kinematic)");
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    
    // Sync geometry
    p_.geometry.wheelbase_m = p_.wheelbase_m;
    p_.geometry.track_width_m = p_.track_width_m;
    p_.geometry.wheel_radius_m = p_.drive.wheel_radius_m;
    
    // Update subsystem parameters
    auto* steer = subsystem_mgr_.find_subsystem("Steer");
    if (steer) {
        static_cast<SteerSubsystem*>(steer)->set_params(p_.steer);
    }
    
    auto* drive = subsystem_mgr_.find_subsystem("Drive");
    if (drive) {
        static_cast<DriveSubsystem*>(drive)->set_params(p_.drive);
    }
    
    auto* battery = subsystem_mgr_.find_subsystem("Battery");
    if (battery) {
        static_cast<BatterySubsystem*>(battery)->set_params(
            p_.battery_params,
            p_.motor_params
        );
    }
    
    // Update TyreSubsystem geometry
    auto* tyre = subsystem_mgr_.find_subsystem("Tyre");
    if (tyre) {
        auto* tyre_sub = static_cast<TyreSubsystem*>(tyre);
        tyre_sub->set_geometry(
            p_.geometry.wheelbase_m,
            p_.geometry.track_width_m,
            p_.drive.mass_kg,
            p_.geometry.cg_height_m,
            p_.geometry.cg_to_front_m,
            p_.geometry.cg_to_rear_m
        );
        tyre_sub->set_tire_radius(p_.drive.wheel_radius_m);
        tyre_sub->set_dynamic_model_enabled(p_.dynamic_config.enabled);
    }
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;
    
    // Sync dynamic model flag with state
    s.dynamic_model_enabled = p_.dynamic_config.enabled;
    s.surface_mu = p_.dynamic_config.surface_mu;

    // Execute all subsystems (pre_step → step → post_step)
    // Order: Steer (50) → Drive (100) → Tyre (105) → Battery (150)
    subsystem_mgr_.step_all(s, cmd, dt_s);

    // ========================================================================
    // Update position/yaw using VehicleBicycleAckermann
    // ========================================================================
    BicycleAckermannParams ap{};
    ap.L_m = p_.wheelbase_m;
    ap.W_m = p_.track_width_m;
    ap.delta_max_rad = (p_.steer.delta_max_deg * 3.141592653589793 / 180.0);
    ap.mu_lat = p_.dynamic_config.surface_mu;  // Use current surface friction
    
    // --- NEW: Pass dynamic model parameters ---
    ap.dynamic_model_enabled = p_.dynamic_config.enabled;
    ap.mass_kg = p_.drive.mass_kg;
    ap.drag_c = p_.drive.drag_c;
    ap.roll_c = p_.drive.roll_c;

    BicycleState2D pose{};
    pose.x_m = s.x_m;
    pose.y_m = s.y_m;
    pose.yaw_rad = s.yaw_rad;

    // Note: VehicleBicycleAckermann::step still takes BatteryPlant reference
    // This is OK - it's wrapped inside BatterySubsystem
    // Future: Could refactor to not need this, but works fine for now
    auto* battery_sub = subsystem_mgr_.find_subsystem("Battery");
    if (battery_sub) {
        // Access internal BatteryPlant through subsystem (requires friend access or getter)
        // For now, bicycle step doesn't actually modify battery, so we can pass a dummy
        BatteryPlant dummy_battery(p_.battery_params, p_.motor_params);
        
        // Pass PlantState for dynamic mode tire forces
        auto res = VehicleBicycleAckermann::step(pose, s.v_mps, s.steer_virtual_rad, ap, dt_s, dummy_battery, s);
        
        s.x_m = res.next.x_m;
        s.y_m = res.next.y_m;
        s.yaw_rad = res.next.yaw_rad;
        
        // In dynamic mode, velocity is updated from tire forces
        if (p_.dynamic_config.enabled) {
            s.v_mps = res.next.speed_mps;
            s.a_long_mps2 = res.a_long_mps2;
        }
    }
}

// ============================================================================
// Dynamic Model Control 
// ============================================================================

void PlantModel::set_dynamic_model_enabled(bool enabled) {
    p_.dynamic_config.enabled = enabled;
    
    // Update TyreSubsystem --. REPLACE WITH WHEEL 
    auto* tyre = subsystem_mgr_.find_subsystem("Tyre");
    if (tyre) {
        static_cast<TyreSubsystem*>(tyre)->set_dynamic_model_enabled(enabled);
    }
    
    LOG_INFO("[PlantModel] Dynamic model: %s", enabled ? "ENABLED (Dugoff)" : "DISABLED (Kinematic)");
}

void PlantModel::set_surface_friction(double mu) {
    p_.dynamic_config.surface_mu = mu;
    
    // Update TyreSubsystem with new friction
    auto* tyre = subsystem_mgr_.find_subsystem("Tyre");
    if (tyre) {
        static_cast<TyreSubsystem*>(tyre)->set_surface_friction(mu);
    }
    
    LOG_INFO("[PlantModel] Surface friction set to mu=%.2f", mu);
}

} // namespace plant