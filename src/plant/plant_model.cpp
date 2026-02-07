// src/plant/plant_model.cpp
// COMPLETE VERSION - Hybrid Torque-Slip Model with Wheel Dynamics
#include "plant/plant_model.hpp"
#include "plant/battery_subsystem/battery_subsystem.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_subsystem.hpp"      // NEW!
#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"  // NEW!
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
    
    LOG_INFO("[PlantModel] Initializing with SubsystemManager architecture");
    LOG_INFO("[PlantModel] Battery: %.1f kWh, Motor: %.1f kW, Mass: %.0f kg",
             p_.battery_params.capacity_kWh, p_.motor_params.max_power_kW, p_.drive.mass_kg);

    // ========================================================================
    // Register Subsystems (auto-sorted by priority)
    // ========================================================================
    
    // Priority 50: SteerSubsystem - Ackermann geometry
    auto steer = std::make_unique<SteerSubsystem>(p_.steer);
    subsystem_mgr_.register_subsystem(std::move(steer));
    
    // Priority 100: DriveSubsystem - Motor/brake torque distribution
    auto drive = std::make_unique<DriveSubsystem>(p_.drive);
    subsystem_mgr_.register_subsystem(std::move(drive));
    
    // Priority 105: WheelSubsystem - Wheel dynamics + Dugoff tire model (NEW!)
    WheelSubsystemParams wheel_params;
    
    // a) Wheel dynamics
    wheel_params.wheel.inertia_kgm2 = 1000.0;  // TODO: Get from config
    wheel_params.wheel.radius_m = p_.drive.wheel_radius_m;
    wheel_params.wheel.omega_max_radps = 50.0;
    wheel_params.wheel.omega_min_radps = -10.0;
    wheel_params.wheel.v_eps_mps = 0.1;
    wheel_params.wheel.omega_eps_radps = 0.01;
    
    // b) Vehicle geometry (for normal load calculation)
    wheel_params.wheelbase_m = p_.wheelbase_m;
    wheel_params.track_width_m = p_.track_width_m;
    wheel_params.cg_height_m = p_.geometry.cg_height_m;
    wheel_params.cg_to_front_m = p_.geometry.cg_to_front_m;
    wheel_params.mass_kg = p_.drive.mass_kg;
    
    // c) Tire parameters (TyreDugoff)
    wheel_params.tyre_params.Cx_base = 280000.0;
    wheel_params.tyre_params.Cy_base = 220000.0;
    wheel_params.tyre_params.Fz_ref = 800000.0;
    wheel_params.tyre_params.load_exponent = 0.50;
    wheel_params.tyre_params.mu_peak = p_.dynamic_config.surface_mu;
    wheel_params.tyre_params.mu_slide = p_.dynamic_config.surface_mu * 0.9;
    wheel_params.tyre_params.sigma_x_max = 0.95;
    wheel_params.tyre_params.sigma_y_max = 0.50;
    wheel_params.tyre_params.v_min = 0.5;
    
    // d) Dynamic mode flag
    wheel_params.dynamic_mode_enabled = p_.dynamic_config.enabled;
    
    auto wheel = std::make_unique<WheelSubsystem>(wheel_params);
    subsystem_mgr_.register_subsystem(std::move(wheel));
    
    LOG_INFO("[PlantModel] WheelSubsystem: m=%.0f kg, L=%.2f m, h_cg=%.2f m, mu=%.2f",
             wheel_params.mass_kg, wheel_params.wheelbase_m, 
             wheel_params.cg_height_m, wheel_params.tyre_params.mu_peak);
    
    // Priority 110: VehicleSubsystem - Longitudinal/yaw/position integration (NEW!)
    VehicleSubsystemParams vehicle_params;
    vehicle_params.mass_kg = p_.drive.mass_kg;
    vehicle_params.wheelbase_m = p_.wheelbase_m;
    vehicle_params.track_width_m = p_.track_width_m;
    vehicle_params.drag_c = p_.drive.drag_c;
    vehicle_params.roll_c = p_.drive.roll_c;
    vehicle_params.v_max_mps = p_.drive.v_max_mps;
    vehicle_params.dynamic_mode_enabled = p_.dynamic_config.enabled;
    
    auto vehicle = std::make_unique<VehicleSubsystem>(vehicle_params);
    subsystem_mgr_.register_subsystem(std::move(vehicle));
    
    // Priority 150: BatterySubsystem - Energy management
    auto battery = std::make_unique<BatterySubsystem>(
        p_.battery_params,
        p_.motor_params
    );
    auto* battery_ptr = battery.get();
    subsystem_mgr_.register_subsystem(std::move(battery));
    
    // Inject battery dependency into DriveSubsystem
    auto* drive_sub = subsystem_mgr_.find_subsystem("Drive");
    if (drive_sub) {
        static_cast<DriveSubsystem*>(drive_sub)->set_battery_subsystem(battery_ptr);
    }
    
    // Initialize all subsystems with default state
    PlantState init_state{};
    init_state.batt_soc_pct = 50.0;
    init_state.batt_v = p_.battery_params.nominal_voltage_v;
    init_state.dynamic_model_enabled = p_.dynamic_config.enabled;
    init_state.surface_mu = p_.dynamic_config.surface_mu;
    subsystem_mgr_.initialize_all(init_state);
    
    LOG_INFO("[PlantModel] Registered %zu subsystems", subsystem_mgr_.subsystem_count());
    LOG_INFO("[PlantModel] Dynamic model: %s", 
             p_.dynamic_config.enabled ? "ENABLED (Dugoff)" : "DISABLED (kinematic)");
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    
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
    
    // TODO: Add set_params for WheelSubsystem and VehicleSubsystem
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;

    // Execute all subsystems in priority order:
    // 50: Steer → δFL, δFR
    // 100: Drive → τ_drive_*, τ_brake_*
    // 105: Wheel → ω, Fx, Fy, Fz, σ, λ
    // 110: Vehicle → v, ψ, x, y
    // 150: Battery → SOC, V, I
    subsystem_mgr_.step_all(s, cmd, dt_s);
}

} // namespace plant