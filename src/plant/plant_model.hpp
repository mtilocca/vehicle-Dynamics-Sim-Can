// src/plant/plant_model.hpp
#pragma once

#include "plant/plant_state.hpp"
#include "plant/subsystem_manager.hpp"
#include "plant/steer_plant.hpp"
#include "plant/drive_plant.hpp"
#include "plant/battery_plant.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct PlantModelParams {
    SteerParams steer{};
    DriveParams drive{};
    BatteryPlantParams battery_params{};
    MotorParams motor_params{};

    double wheelbase_m = 2.8;
    double track_width_m = 1.6;
};

/**
 * PlantModel - Main vehicle physics orchestrator
 * 
 * Now uses SubsystemManager for scalable subsystem architecture.
 * Subsystems are registered by priority and executed automatically.
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

private:
    PlantModelParams p_;
    SubsystemManager subsystem_mgr_;
};

} // namespace plant