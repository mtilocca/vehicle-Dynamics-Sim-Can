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
    // ---------------------------------------------------------------------
    // Geometry wiring (Steer subsystem depends on this)
    // ---------------------------------------------------------------------
    p_.steer.wheelbase_m   = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    LOG_INFO("[PlantModel] Initializing with SubsystemManager architecture");
    LOG_INFO("[PlantModel] Battery: %.1f kWh | Motor: %.1f kW | Mass: %.0f kg",
             p_.battery_params.capacity_kWh,
             p_.motor_params.max_power_kW,
             p_.drive.mass_kg);

    // =====================================================================
    // Priority 50 — Steering (Ackermann)
    // =====================================================================
    {
        auto steer = std::make_unique<SteerSubsystem>(p_.steer);
        subsystem_mgr_.register_subsystem(std::move(steer));
    }

    // =====================================================================
    // Priority 100 — Drive (motor + brake torque)
    // =====================================================================
    {
        auto drive = std::make_unique<DriveSubsystem>(p_.drive);
        subsystem_mgr_.register_subsystem(std::move(drive));
    }

    // =====================================================================
    // Priority 105 — Wheel subsystem (wheel dynamics + tire model)
    // =====================================================================
    {
        WheelSubsystemParams wheel_params{};

        // -----------------------------
        // Vehicle / geometry
        // -----------------------------
        wheel_params.mass_kg        = p_.drive.mass_kg;
        wheel_params.wheelbase_m    = p_.wheelbase_m;
        wheel_params.track_width_m  = p_.track_width_m;
        wheel_params.cg_height_m    = p_.geometry.cg_height_m;
        wheel_params.cg_to_front_m  = p_.geometry.cg_to_front_m;

        // -----------------------------
        // Wheel dynamics parameters
        // -----------------------------
        wheel_params.wheel.inertia_kgm2   = 1000.0;                 // TODO: move to YAML
        wheel_params.wheel.radius_m       = p_.drive.wheel_radius_m;
        wheel_params.wheel.omega_max_radps = 50.0;
        wheel_params.wheel.omega_min_radps = -10.0;
        wheel_params.wheel.v_eps_mps       = 0.1;
        wheel_params.wheel.omega_eps_radps = 0.01;

        // -----------------------------
        // Dynamic tire model config
        // -----------------------------
        wheel_params.surface_mu       = p_.dynamic_config.surface_mu;
        wheel_params.dynamic_enabled  = p_.dynamic_config.enabled;

        auto wheel = std::make_unique<WheelSubsystem>(wheel_params);
        subsystem_mgr_.register_subsystem(std::move(wheel));

        LOG_INFO(
            "[PlantModel] WheelSubsystem: m=%.0f kg | L=%.2f m | h_cg=%.2f m | mu=%.2f",
            wheel_params.mass_kg,
            wheel_params.wheelbase_m,
            wheel_params.cg_height_m,
            wheel_params.surface_mu
        );
    }

    // =====================================================================
    // Priority 110 — Vehicle integration (v, yaw, x, y)
    // =====================================================================
    {
        VehicleParams vehicle_params{};
        vehicle_params.mass_kg     = p_.drive.mass_kg;
        vehicle_params.wheelbase_m = p_.wheelbase_m;
        vehicle_params.drag_c      = p_.drive.drag_c;
        vehicle_params.roll_c      = p_.drive.roll_c;
        vehicle_params.v_max_mps   = p_.drive.v_max_mps;
        vehicle_params.v_stop_eps  = p_.drive.v_stop_eps;

        auto vehicle = std::make_unique<VehicleSubsystem>(vehicle_params);
        subsystem_mgr_.register_subsystem(std::move(vehicle));
    }

    // =====================================================================
    // Priority 150 — Battery
    // =====================================================================
    {
        auto battery = std::make_unique<BatterySubsystem>(
            p_.battery_params,
            p_.motor_params
        );
        auto* battery_ptr = battery.get();
        subsystem_mgr_.register_subsystem(std::move(battery));

        // Inject dependency into DriveSubsystem
        auto* drive_sub = subsystem_mgr_.find_subsystem("Drive");
        if (drive_sub) {
            static_cast<DriveSubsystem*>(drive_sub)
                ->set_battery_subsystem(battery_ptr);
        }
    }

    // =====================================================================
    // Initial state
    // =====================================================================
    PlantState init_state{};
    init_state.batt_soc_pct = 50.0;
    init_state.batt_v = p_.battery_params.nominal_voltage_v;
    init_state.dynamic_model_enabled = p_.dynamic_config.enabled;
    init_state.surface_mu = p_.dynamic_config.surface_mu;

    subsystem_mgr_.initialize_all(init_state);

    LOG_INFO("[PlantModel] Registered %zu subsystems",
             subsystem_mgr_.subsystem_count());
    LOG_INFO("[PlantModel] Dynamic model: %s",
             p_.dynamic_config.enabled ? "ENABLED (Dugoff)" : "DISABLED (kinematic)");
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;

    p_.steer.wheelbase_m    = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    if (auto* steer = subsystem_mgr_.find_subsystem("Steer")) {
        static_cast<SteerSubsystem*>(steer)->set_params(p_.steer);
    }

    if (auto* drive = subsystem_mgr_.find_subsystem("Drive")) {
        static_cast<DriveSubsystem*>(drive)->set_params(p_.drive);
    }

    if (auto* battery = subsystem_mgr_.find_subsystem("Battery")) {
        static_cast<BatterySubsystem*>(battery)->set_params(
            p_.battery_params,
            p_.motor_params
        );
    }

    // TODO: add hot-update for WheelSubsystem + VehicleSubsystem
}

void PlantModel::step(PlantState& s,
                      const sim::ActuatorCmd& cmd,
                      double dt_s)
{
    s.t_s += dt_s;

    // Execution order:
    //  50  Steer
    // 100  Drive
    // 105  Wheel
    // 110  Vehicle
    // 150  Battery
    subsystem_mgr_.step_all(s, cmd, dt_s);
}

} // namespace plant
