#pragma once

#include "plant_state.hpp"
#include "drive_plant.hpp"
#include "steer_plant.hpp"
#include "battery_plant.hpp"  // Include BatteryPlant class for energy management

namespace sim { struct ActuatorCmd; }

namespace plant {

struct PlantModelParams {
    // Pass-through convenience so you can tune from one place
    SteerParams steer{};
    DriveParams drive{};
    BatteryPlantParams battery_params{};
    MotorParams motor_params{};

    // Kinematic geometry for pose integration
    double wheelbase_m = 2.8;
    double track_width_m = 1.6;
};

class PlantModel {
public:
    explicit PlantModel(PlantModelParams p = {});

    const PlantModelParams& params() const { return p_; }
    void set_params(const PlantModelParams& p);

    // Fixed deterministic order:
    //  1) steering update (virtual steer + Ackermann)
    //  2) drive update (speed)
    //  3) bicycle kinematic pose integration (x,y,yaw)
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

private:
    PlantModelParams p_;
    SteerPlant steer_;
    DrivePlant drive_;
    BatteryPlant battery_;  // Instance of BatteryPlant to manage energy

};

} // namespace plant
