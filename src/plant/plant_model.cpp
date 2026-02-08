// src/plant/plant_model.cpp
// COMPLETE VERSION - Hybrid Torque-Slip Model with Wheel Dynamics
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
    // Initialize geometry parameters used by SteerSubsystem
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    LOG_INFO("[PlantModel] Initializing with SubsystemManager architecture");
    LOG_INFO("[PlantModel] Battery: %.1f kWh, Motor: %.1f kW, Mass: %.0f kg",
             p_.battery_params.capacity_kWh, p_.motor_params.max_power_kW, p_.drive.mass_kg);

    // ========================================================================
    // Register Subsystems (auto-sorted by priority)
    // ========================================================================

    // Priority 50: SteerSubsystem - Ackermann geometry
    {
        auto steer = std::make_unique<SteerSubsystem>(p_.steer);
        subsystem_mgr_.register_subsystem(std::move(steer));
    }

    // Priority 100: DriveSubsystem - Motor/brake torque distribution
    {
        auto drive = std::make_unique<DriveSubsystem>(p_.drive);
        subsystem_mgr_.register_subsystem(std::move(drive));
    }

    // Priority 105: WheelSubsystem - Wheel dynamics + Dugoff tire model
    {
        WheelSubsystemParams wheel_params;

        // Wheel dynamics parameters (single-wheel model reused 4x internally)
        wheel_params.wheel.inertia_kgm2 = 1000.0;                 // TODO: load from YAML if desired
        wheel_params.wheel.radius_m = p_.drive.wheel_radius_m;    // match drivetrain wheel radius
        wheel_params.wheel.omega_max_radps = 50.0;
        wheel_params.wheel.omega_min_radps = -10.0;
        wheel_params.wheel.v_eps_mps = 0.1;
        wheel_params.wheel.omega_eps_radps = 0.01;

        // Vehicle geometry (needed for 3-DOF wheel kinematics and load transfer)
        wheel_params.mass_kg = p_.drive.mass_kg;
        wheel_params.wheelbase_m = p_.wheelbase_m;
        wheel_params.track_m = p_.track_width_m;
        wheel_params.cg_height_m = p_.geometry.cg_height_m;
        wheel_params.cg_to_front_m = p_.geometry.cg_to_front_m;
        wheel_params.wheel_radius_m = p_.drive.wheel_radius_m;

        // Surface friction quick overrides (WheelSubsystem owns these)
        wheel_params.mu_peak  = p_.dynamic_config.surface_mu;
        wheel_params.mu_slide = p_.dynamic_config.surface_mu * 0.90;

        // Tyre Dugoff parameters (keep consistent with your tyre model header)
        wheel_params.tyre_params.Cx_base = 280000.0;
        wheel_params.tyre_params.Cy_base = 220000.0;
        wheel_params.tyre_params.Fz_ref = 800000.0;
        wheel_params.tyre_params.load_exponent = 0.50;

        wheel_params.tyre_params.mu_peak  = wheel_params.mu_peak;
        wheel_params.tyre_params.mu_slide = wheel_params.mu_slide;

        wheel_params.tyre_params.sigma_x_max = 0.95;
        wheel_params.tyre_params.sigma_y_max = 0.50;
        wheel_params.tyre_params.v_min = 0.5;

        // Enable/disable dynamic wheel+tire mode
        wheel_params.dynamic_mode_enabled = p_.dynamic_config.enabled;

        auto wheel = std::make_unique<WheelSubsystem>(wheel_params);
        subsystem_mgr_.register_subsystem(std::move(wheel));

        LOG_INFO("[PlantModel] WheelSubsystem: L=%.2f m, track=%.2f m, h_cg=%.2f m, mu_peak=%.2f (dyn=%s)",
                 wheel_params.wheelbase_m,
                 wheel_params.track_m,
                 wheel_params.cg_height_m,
                 wheel_params.mu_peak,
                 wheel_params.dynamic_mode_enabled ? "ON" : "OFF");
    }

    // Priority 110: VehicleSubsystem - integrates vehicle motion (3-DOF)
    {
        VehicleParams vehicle_params;
        vehicle_params.mass_kg = p_.drive.mass_kg;
        vehicle_params.wheelbase_m = p_.wheelbase_m;
        vehicle_params.track_m = p_.track_width_m;
        vehicle_params.cg_to_front_m = p_.geometry.cg_to_front_m;
        vehicle_params.yaw_inertia_kgm2 = p_.geometry.yaw_inertia_kgm2;
        vehicle_params.drag_c = p_.drive.drag_c;
        vehicle_params.roll_c = p_.drive.roll_c;
        vehicle_params.v_max_mps = p_.drive.v_max_mps;
        vehicle_params.v_stop_eps = p_.drive.v_stop_eps;

        auto vehicle = std::make_unique<VehicleSubsystem>(vehicle_params);
        subsystem_mgr_.register_subsystem(std::move(vehicle));
    }

    // Priority 150: BatterySubsystem - energy management
    BatterySubsystem* battery_ptr = nullptr;
    {
        auto battery = std::make_unique<BatterySubsystem>(p_.battery_params, p_.motor_params);
        battery_ptr = battery.get();
        subsystem_mgr_.register_subsystem(std::move(battery));
    }

    // Inject battery dependency into DriveSubsystem (if present)
    {
        auto* drive_sub = subsystem_mgr_.find_subsystem("Drive");
        if (drive_sub && battery_ptr) {
            static_cast<DriveSubsystem*>(drive_sub)->set_battery_subsystem(battery_ptr);
        }
    }

    // Initialize all subsystems with default state
    {
        PlantState init_state{};
        init_state.batt_soc_pct = 50.0;
        init_state.batt_v = p_.battery_params.nominal_voltage_v;
        init_state.dynamic_model_enabled = p_.dynamic_config.enabled;
        init_state.surface_mu = p_.dynamic_config.surface_mu;

        subsystem_mgr_.initialize_all(init_state);
    }

    LOG_INFO("[PlantModel] Registered %zu subsystems", subsystem_mgr_.subsystem_count());
    LOG_INFO("[PlantModel] Dynamic model: %s",
             p_.dynamic_config.enabled ? "ENABLED (Dugoff + wheel dynamics)" : "DISABLED");
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    // Steer
    if (auto* steer = subsystem_mgr_.find_subsystem("Steer")) {
        static_cast<SteerSubsystem*>(steer)->set_params(p_.steer);
    }

    // Drive
    if (auto* drive = subsystem_mgr_.find_subsystem("Drive")) {
        static_cast<DriveSubsystem*>(drive)->set_params(p_.drive);
    }

    // Wheel (rebuild WheelSubsystemParams based on new PlantModelParams)
    if (auto* wheel = subsystem_mgr_.find_subsystem("Wheel")) {
        WheelSubsystemParams wheel_params;

        wheel_params.wheel.inertia_kgm2 = 1000.0;               // TODO: load from YAML
        wheel_params.wheel.radius_m = p_.drive.wheel_radius_m;
        wheel_params.wheel.omega_max_radps = 50.0;
        wheel_params.wheel.omega_min_radps = -10.0;
        wheel_params.wheel.v_eps_mps = 0.1;
        wheel_params.wheel.omega_eps_radps = 0.01;

        wheel_params.mass_kg = p_.drive.mass_kg;
        wheel_params.wheelbase_m = p_.wheelbase_m;
        wheel_params.track_m = p_.track_width_m;
        wheel_params.cg_height_m = p_.geometry.cg_height_m;
        wheel_params.cg_to_front_m = p_.geometry.cg_to_front_m;
        wheel_params.wheel_radius_m = p_.drive.wheel_radius_m;

        wheel_params.mu_peak  = p_.dynamic_config.surface_mu;
        wheel_params.mu_slide = p_.dynamic_config.surface_mu * 0.90;

        wheel_params.tyre_params.Cx_base = 280000.0;
        wheel_params.tyre_params.Cy_base = 220000.0;
        wheel_params.tyre_params.Fz_ref = 800000.0;
        wheel_params.tyre_params.load_exponent = 0.50;
        wheel_params.tyre_params.mu_peak  = wheel_params.mu_peak;
        wheel_params.tyre_params.mu_slide = wheel_params.mu_slide;
        wheel_params.tyre_params.sigma_x_max = 0.95;
        wheel_params.tyre_params.sigma_y_max = 0.50;
        wheel_params.tyre_params.v_min = 0.5;

        wheel_params.dynamic_mode_enabled = p_.dynamic_config.enabled;

        static_cast<WheelSubsystem*>(wheel)->set_params(wheel_params);
    }

    // Vehicle (3-DOF)
    if (auto* vehicle = subsystem_mgr_.find_subsystem("Vehicle")) {
        VehicleParams vehicle_params;
        vehicle_params.mass_kg = p_.drive.mass_kg;
        vehicle_params.wheelbase_m = p_.wheelbase_m;
        vehicle_params.track_m = p_.track_width_m;
        vehicle_params.cg_to_front_m = p_.geometry.cg_to_front_m;
        vehicle_params.yaw_inertia_kgm2 = p_.geometry.yaw_inertia_kgm2;
        vehicle_params.drag_c = p_.drive.drag_c;
        vehicle_params.roll_c = p_.drive.roll_c;
        vehicle_params.v_max_mps = p_.drive.v_max_mps;
        vehicle_params.v_stop_eps = p_.drive.v_stop_eps;

        static_cast<VehicleSubsystem*>(vehicle)->set_params(vehicle_params);
    }

    // Battery
    if (auto* battery = subsystem_mgr_.find_subsystem("Battery")) {
        static_cast<BatterySubsystem*>(battery)->set_params(p_.battery_params, p_.motor_params);
    }
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;

    // Update gear position from CAN command
    s.gear_position = cmd.gear_position;

    // Execute all subsystems in priority order:
    // 50:  Steer   → δFL, δFR
    // 100: Drive   → τ_drive_*, τ_brake_*
    // 105: Wheel   → ω, Fx, Fy, Fz, σ, λ
    // 110: Vehicle → v, ψ, x, y
    // 150: Battery → SOC, V, I
    subsystem_mgr_.step_all(s, cmd, dt_s);
}

} // namespace plant
