#pragma once

#include "plant_state.hpp"
#include "drive_plant.hpp"
#include "steer_plant.hpp"
#include "battery_plant.hpp"

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

class PlantModel {
public:
    explicit PlantModel(PlantModelParams p = {});

    const PlantModelParams& params() const { return p_; }
    void set_params(const PlantModelParams& p);

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

private:
    PlantModelParams p_;
    SteerPlant steer_;
    DrivePlant drive_;
    BatteryPlant battery_;  
};

} // namespace plant
