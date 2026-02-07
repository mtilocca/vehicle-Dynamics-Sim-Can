// src/plant/plant_model.cpp
//
// PlantModel - Main vehicle physics orchestrator (HYBRID MODEL)
//
// Execution order (by subsystem priority):
//   50  - SteerSubsystem     (δFL, δFR from Ackermann)
//  100  - DriveSubsystem     (τdrive_*, τbrake_*)
//  105  - WheelSubsystem     (ω, Fx from WheelDynamics + TyreDugoff)
//  110  - VehicleSubsystem   (v, ψ, x, y integration)
//  150  - BatterySubsystem   (Energy tracking)

#include "plant/plant_model.hpp"
#include "plant/battery_subsystem/battery_subsystem.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
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
    
    // 2. DriveSubsystem (Priority 100 - Dynamics)
    auto drive = std::make_unique<DriveSubsystem>(p_.drive);
    subsystem_mgr_.register_subsystem(std::move(drive));
    
    // 3. WheelSubsystem (Priority 105 - Wheel Dynamics & Tire Forces)
    WheelSubsystemParams wheel_params;
    
    // a) Wheel dynamics parameters
    wheel_params.wheel.inertia_kgm2 = 1000.0;  // XCMG XDE320 wheel+tire inertia
    wheel_params.wheel.radius_m = p_.drive.wheel_radius_m;
    wheel_params.wheel.omega_max_radps = 50.0;
    wheel_params.wheel.omega_min_radps = -10.0;
    wheel_params.wheel.v_eps_mps = 0.1;
    wheel_params.wheel.omega_eps_radps = 0.01;
    
    // b) Vehicle geometry (for normal load calculation)
    wheel_params.wheelbase_m = p_.geometry.wheelbase_m;
    wheel_params.track_width_m = p_.geometry.track_width_m;
    wheel_params.cg_height_m = p_.geometry.cg_height_m;
    wheel_params.cg_to_front_m = p_.geometry.cg_to_front_m;
    wheel_params.mass_kg = p_.drive.mass_kg;
    
    // c) Tire parameters (TyreDugoff)
    wheel_params.tyre_params.Cx_base = 280000.0;     // Longitudinal stiffness [N]
    wheel_params.tyre_params.Cy_base = 220000.0;     // Lateral stiffness [N]
    wheel_params.tyre_params.Fz_ref = 800000.0;      // Reference normal load [N]
    wheel_params.tyre_params.load_exponent = 0.50;   // Load scaling exponent
    wheel_params.tyre_params.mu_peak = p_.dynamic_config.surface_mu;
    wheel_params.tyre_params.mu_slide = p_.dynamic_config.surface_mu * 0.9;
    wheel_params.tyre_params.velocity_fade_enabled = false;
    wheel_params.tyre_params.sigma_x_max = 0.95;
    wheel_params.tyre_params.sigma_y_max = 0.50;
    wheel_params.tyre_params.v_min = 0.5;
    
    // d) Dynamic mode flag
    wheel_params.dynamic_mode_enabled = p_.dynamic_config.enabled;
    
    auto wheel = std::make_unique<WheelSubsystem>(wheel_params);
    subsystem_mgr_.register_subsystem(std::move(wheel));
    
    // 4. VehicleSubsystem (Priority 110 - Vehicle Dynamics Integration)
    VehicleParams vehicle_params;
    vehicle_params.mass_kg = p_.drive.mass_kg;
    vehicle_params.wheelbase_m = p_.geometry.wheelbase_m;
    vehicle_params.drag_c = p_.drive.drag_c;
    vehicle_params.roll_c = p_.drive.roll_c;
    vehicle_params.v_stop_eps = p_.drive.v_stop_eps;
    vehicle_params.v_max_mps = p_.drive.v_max_mps;
    
    auto vehicle = std::make_unique<VehicleSubsystem>(vehicle_params);
    subsystem_mgr_.register_subsystem(std::move(vehicle));
    
    // 5. BatterySubsystem (Priority 150 - Energy)
    auto battery = std::make_unique<BatterySubsystem>(
        p_.battery_params,
        p_.motor_params
    );
    auto* battery_ptr = battery.get();
    subsystem_mgr_.register_subsystem(std::move(battery));
    
    // Connect battery to drive
    auto* drive_sub = subsystem_mgr_.find_subsystem("Drive");
    if (drive_sub) {
        static_cast<DriveSubsystem*>(drive_sub)->set_battery_subsystem(battery_ptr);
    }
    
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
    
    auto* vehicle = subsystem_mgr_.find_subsystem("Vehicle");
    if (vehicle) {
        VehicleParams vp;
        vp.mass_kg = p_.drive.mass_kg;
        vp.wheelbase_m = p_.geometry.wheelbase_m;
        vp.drag_c = p_.drive.drag_c;
        vp.roll_c = p_.drive.roll_c;
        vp.v_stop_eps = p_.drive.v_stop_eps;
        vp.v_max_mps = p_.drive.v_max_mps;
        static_cast<VehicleSubsystem*>(vehicle)->set_params(vp);
    }
    
    // Update WheelSubsystem geometry and tire params
    auto* wheel = subsystem_mgr_.find_subsystem("Wheel");
    if (wheel) {
        auto* wheel_sub = static_cast<WheelSubsystem*>(wheel);
        
        // Get current params, update selective fields
        WheelSubsystemParams wp = wheel_sub->get_params();
        wp.wheelbase_m = p_.geometry.wheelbase_m;
        wp.track_width_m = p_.geometry.track_width_m;
        wp.cg_height_m = p_.geometry.cg_height_m;
        wp.cg_to_front_m = p_.geometry.cg_to_front_m;
        wp.mass_kg = p_.drive.mass_kg;
        wp.wheel.radius_m = p_.drive.wheel_radius_m;
        wp.tyre_params.mu_peak = p_.dynamic_config.surface_mu;
        wp.tyre_params.mu_slide = p_.dynamic_config.surface_mu * 0.9;
        wp.dynamic_mode_enabled = p_.dynamic_config.enabled;
        
        wheel_sub->set_params(wp);
    }
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;
    
    // Sync dynamic model flag with state
    s.dynamic_model_enabled = p_.dynamic_config.enabled;
    s.surface_mu = p_.dynamic_config.surface_mu;

    // ========================================================================
    // Execute All Subsystems
    // ========================================================================
    // Order: Steer (50) → Drive (100) → Wheel (105) → Vehicle (110) → Battery (150)
    //
    // Execution flow:
    //   1. SteerSubsystem  → δFL, δFR (Ackermann geometry)
    //   2. DriveSubsystem  → τdrive_*, τbrake_* (torque distribution)
    //   3. WheelSubsystem  → ω, Fx (wheel dynamics + Dugoff tire model)
    //   4. VehicleSubsystem → v, ψ, x, y (longitudinal & yaw integration)
    //   5. BatterySubsystem → SOC, V, I (energy tracking)
    
    subsystem_mgr_.step_all(s, cmd, dt_s);
    
    // ========================================================================
    // DONE! All integration handled by subsystems
    // ========================================================================
    // No need for VehicleBicycleAckermann - VehicleSubsystem replaced it
}

// ============================================================================
// Dynamic Model Control 
// ============================================================================

void PlantModel::set_dynamic_model_enabled(bool enabled) {
    p_.dynamic_config.enabled = enabled;
    
    // Update WheelSubsystem dynamic mode
    auto* wheel = subsystem_mgr_.find_subsystem("Wheel");
    if (wheel) {
        static_cast<WheelSubsystem*>(wheel)->set_dynamic_mode(enabled);
    }
    
    LOG_INFO("[PlantModel] Dynamic model: %s", enabled ? "ENABLED (Dugoff)" : "DISABLED (Kinematic)");
}

void PlantModel::set_surface_friction(double mu) {
    p_.dynamic_config.surface_mu = mu;
    
    // Update WheelSubsystem with new friction
    auto* wheel = subsystem_mgr_.find_subsystem("Wheel");
    if (wheel) {
        static_cast<WheelSubsystem*>(wheel)->set_surface_mu(mu, mu * 0.9);
    }
    
    LOG_INFO("[PlantModel] Surface friction set to mu=%.2f", mu);
}

} // namespace plant