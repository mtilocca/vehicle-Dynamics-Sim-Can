// zephyr/src/plant_model_zephyr.cpp
// Zephyr-only PlantModel constructor + set_params.
// Replaces src/plant/plant_model.cpp in the Zephyr build.
//
// Key difference from the host version:
//   - Subsystems stored by value inside PlantModel (no heap, no unique_ptr)
//   - SubsystemManager holds non-owning raw pointers sorted by priority
//   - All XCMG XDE320 params are identical to the host build; only ownership
//     model differs.

#include "plant/plant_model.hpp"
#include "plant/battery_subsystem/battery_subsystem.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"

LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);

namespace plant {

// ── helpers ───────────────────────────────────────────────────────────────────

static WheelSubsystemParams make_wheel_params(const PlantModelParams& p)
{
    WheelSubsystemParams wp;
    wp.wheel.inertia_kgm2      = 1000.0;
    wp.wheel.radius_m          = p.drive.wheel_radius_m;
    wp.wheel.omega_max_radps   = 50.0;
    wp.wheel.omega_min_radps   = -10.0;
    wp.wheel.v_eps_mps         = 0.1;
    wp.wheel.omega_eps_radps   = 0.01;

    wp.mass_kg         = p.drive.mass_kg;
    wp.wheelbase_m     = p.wheelbase_m;
    wp.track_m         = p.track_width_m;
    wp.cg_height_m     = p.geometry.cg_height_m;
    wp.cg_to_front_m   = p.geometry.cg_to_front_m;
    wp.wheel_radius_m  = p.drive.wheel_radius_m;

    wp.mu_peak  = p.dynamic_config.surface_mu;
    wp.mu_slide = p.dynamic_config.surface_mu * 0.90;

    wp.tyre_params.Cx_base        = 280000.0;
    wp.tyre_params.Cy_base        = 220000.0;
    wp.tyre_params.Fz_ref         = 800000.0;
    wp.tyre_params.load_exponent  = 0.50;
    wp.tyre_params.mu_peak        = wp.mu_peak;
    wp.tyre_params.mu_slide       = wp.mu_slide;
    wp.tyre_params.sigma_x_max    = 0.95;
    wp.tyre_params.sigma_y_max    = 0.50;
    wp.tyre_params.v_min          = 0.5;

    wp.dynamic_mode_enabled = p.dynamic_config.enabled;
    return wp;
}

static VehicleParams make_vehicle_params(const PlantModelParams& p)
{
    VehicleParams vp;
    vp.mass_kg            = p.drive.mass_kg;
    vp.wheelbase_m        = p.wheelbase_m;
    vp.track_m            = p.track_width_m;
    vp.cg_to_front_m      = p.geometry.cg_to_front_m;
    vp.yaw_inertia_kgm2   = p.geometry.yaw_inertia_kgm2;
    vp.drag_c             = p.drive.drag_c;
    vp.roll_c             = p.drive.roll_c;
    vp.v_max_mps          = p.drive.v_max_mps;
    vp.v_stop_eps         = p.drive.v_stop_eps;
    return vp;
}

// ── PlantModel constructor ────────────────────────────────────────────────────

PlantModel::PlantModel(PlantModelParams p)
    : p_(p)
    , steer_(p.steer)
    , drive_(p.drive)
    , wheel_(make_wheel_params(p))
    , vehicle_(make_vehicle_params(p))
    , battery_(p.battery_params, p.motor_params)
{
    p_.steer.wheelbase_m   = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    LOG_INF("[PlantModel] Zephyr build — registering %d subsystems by value",
            (int)SubsystemManager::MAX_SUBSYSTEMS);

    // Register raw pointers — manager does NOT take ownership
    subsystem_mgr_.register_subsystem(&steer_);
    subsystem_mgr_.register_subsystem(&drive_);
    subsystem_mgr_.register_subsystem(&wheel_);
    subsystem_mgr_.register_subsystem(&vehicle_);
    subsystem_mgr_.register_subsystem(&battery_);

    // Inject battery into drive subsystem for power limiting
    drive_.set_battery_subsystem(&battery_);

    // Initialize all subsystems with default PlantState
    PlantState init{};
    init.batt_soc_pct           = 50.0;
    init.batt_v                 = p_.battery_params.nominal_voltage_v;
    init.dynamic_model_enabled  = p_.dynamic_config.enabled;
    init.surface_mu             = p_.dynamic_config.surface_mu;
    subsystem_mgr_.initialize_all(init);

    LOG_INF("[PlantModel] Ready: %zu subsystems, dynamic=%s",
            subsystem_mgr_.subsystem_count(),
            p_.dynamic_config.enabled ? "ON" : "OFF");
}

// ── PlantModel::set_params ────────────────────────────────────────────────────

void PlantModel::set_params(const PlantModelParams& p)
{
    p_ = p;
    p_.steer.wheelbase_m   = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    steer_.set_params(p_.steer);
    drive_.set_params(p_.drive);
    wheel_.set_params(make_wheel_params(p_));

    VehicleParams vp = make_vehicle_params(p_);
    vehicle_.set_params(vp);

    battery_.set_params(p_.battery_params, p_.motor_params);
}

// ── PlantModel::step ──────────────────────────────────────────────────────────

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s)
{
    s.t_s += dt_s;
    s.gear_position = cmd.gear_position;
    subsystem_mgr_.step_all(s, cmd, dt_s);
}

} // namespace plant
