// src/plant/plant_model.cpp
// PlantModel - Simplified 3-subsystem vehicle physics orchestrator

#include "plant/plant_model.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"

namespace plant {

PlantModel::PlantModel(PlantModelParams p)
    : p_(p),
      subsystem_mgr_()
{
    // Wire geometry into steer params
    p_.steer.wheelbase_m   = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;

    // Wire geometry into drive params
    p_.drive.cg_to_front_m = p_.geometry.cg_to_front_m;
    p_.drive.wheelbase_m   = p_.wheelbase_m;

    LOG_INFO("[PlantModel] Initializing: mass=%.0f kg, L=%.2f m, lf=%.2f m, mu=%.2f",
             p_.drive.mass_kg, p_.wheelbase_m,
             p_.geometry.cg_to_front_m, p_.drive.mu_surface);

    // Priority 50: Steer → δFL, δFR
    subsystem_mgr_.register_subsystem(std::make_unique<SteerSubsystem>(p_.steer));

    // Priority 100: Drive → Fx/Fy/Fz per wheel, wheel speeds
    subsystem_mgr_.register_subsystem(std::make_unique<DriveSubsystem>(p_.drive));

    // Priority 110: Vehicle → 3-DOF rigid body integration
    {
        VehicleParams vp;
        vp.mass_kg          = p_.drive.mass_kg;
        vp.wheelbase_m      = p_.wheelbase_m;
        vp.track_m          = p_.track_width_m;
        vp.cg_to_front_m    = p_.geometry.cg_to_front_m;
        vp.yaw_inertia_kgm2 = p_.geometry.yaw_inertia_kgm2;
        vp.drag_c           = p_.drive.drag_c;
        vp.roll_c           = p_.drive.roll_c;
        vp.v_max_mps        = p_.drive.v_max_mps;      // forward limit (60 km/h)
        vp.v_max_rev_mps    = 20.0 / 3.6;              // reverse limit (20 km/h)
        vp.v_lat_max_mps    = 1.0;                     // vy clamp (≈8° sideslip at 7 m/s)
        vp.v_stop_eps       = p_.drive.v_stop_eps;
        vp.v_kinematic_blend_mps = p_.drive.v_kinematic_blend_mps;

        subsystem_mgr_.register_subsystem(std::make_unique<VehicleSubsystem>(vp));
    }

    // Initialize all subsystems with default state
    PlantState init_state{};
    subsystem_mgr_.initialize_all(init_state);

    LOG_INFO("[PlantModel] %zu subsystems registered (Steer:50, Drive:100, Vehicle:110)",
             subsystem_mgr_.subsystem_count());
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m   = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    p_.drive.cg_to_front_m = p_.geometry.cg_to_front_m;
    p_.drive.wheelbase_m   = p_.wheelbase_m;

    if (auto* steer = subsystem_mgr_.find_subsystem("Steer")) {
        static_cast<SteerSubsystem*>(steer)->set_params(p_.steer);
    }
    if (auto* drive = subsystem_mgr_.find_subsystem("Drive")) {
        static_cast<DriveSubsystem*>(drive)->set_params(p_.drive);
    }
    if (auto* vehicle = subsystem_mgr_.find_subsystem("Vehicle")) {
        VehicleParams vp;
        vp.mass_kg          = p_.drive.mass_kg;
        vp.wheelbase_m      = p_.wheelbase_m;
        vp.track_m          = p_.track_width_m;
        vp.cg_to_front_m    = p_.geometry.cg_to_front_m;
        vp.yaw_inertia_kgm2 = p_.geometry.yaw_inertia_kgm2;
        vp.drag_c           = p_.drive.drag_c;
        vp.roll_c           = p_.drive.roll_c;
        vp.v_max_mps        = p_.drive.v_max_mps;      // forward limit (60 km/h)
        vp.v_max_rev_mps    = 20.0 / 3.6;              // reverse limit (20 km/h)
        vp.v_lat_max_mps    = 1.0;                     // vy clamp (≈8° sideslip at 7 m/s)
        vp.v_stop_eps       = p_.drive.v_stop_eps;
        vp.v_kinematic_blend_mps = p_.drive.v_kinematic_blend_mps;
        static_cast<VehicleSubsystem*>(vehicle)->set_params(vp);
    }
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;
    s.gear_position = cmd.gear_position;
    // Execute: Steer(50) → Drive(100) → Vehicle(110)
    subsystem_mgr_.step_all(s, cmd, dt_s);
}

} // namespace plant
