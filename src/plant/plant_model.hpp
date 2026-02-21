// src/plant/plant_model.hpp
#pragma once

#include "plant/plant_main/plant_state.hpp"
#include "plant/subsystem_manager/subsystem_manager.hpp"
#include "plant/steer_subsystem/steer_plant.hpp"
#include "plant/drive_subsystem/drive_plant.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

// CG geometry — used by VehicleSubsystem (3-DOF) and DriveSubsystem (weight distribution)
struct VehicleGeometry {
    double cg_height_m      = 3.20;
    double cg_to_front_m    = 2.52;   // lf
    double cg_to_rear_m     = 3.78;   // lr (= wheelbase - lf)
    double yaw_inertia_kgm2 = 8500000.0;
};

struct PlantModelParams {
    SteerParams steer{};
    DriveParams drive{};

    double wheelbase_m   = 6.30;
    double track_width_m = 7.20;

    VehicleGeometry geometry{};
};

/**
 * PlantModel - Vehicle physics orchestrator
 *
 * Subsystem execution order (priority-based):
 *   50:  SteerSubsystem   → δFL, δFR (Ackermann geometry)
 *  100:  DriveSubsystem   → Fx/Fy/Fz per wheel, wheel speeds (simple model)
 *  110:  VehicleSubsystem → v, ψ, x, y (3-DOF rigid body)
 */
class PlantModel {
public:
    explicit PlantModel(PlantModelParams p = {});

    const PlantModelParams& params() const { return p_; }
    void set_params(const PlantModelParams& p);

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    SubsystemManager& subsystem_manager() { return subsystem_mgr_; }

private:
    PlantModelParams p_;
    SubsystemManager subsystem_mgr_;
};

} // namespace plant
