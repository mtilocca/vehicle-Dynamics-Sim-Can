#pragma once

#include <algorithm>
#include "plant/plant_state.hpp"
#include "battery_plant.hpp"  // Include BatteryPlant class for energy management

namespace sim { struct ActuatorCmd; }

namespace plant {

struct DriveParams {
    double mass_kg = 1800.0;
    double wheel_radius_m = 0.33;
    double drag_c = 0.35;   // N per (m/s)^2
    double roll_c = 40.0;   // N (Coulomb-ish rolling resistance magnitude)
    double motor_torque_max_nm = 4000.0; // motor shaft torque clamp
    double brake_torque_max_nm = 4000.0; // wheel brake torque at tire (magnitude)
    double gear_ratio = 9.0;
    double drivetrain_eff = 0.92; 
    double motor_power_max_w = 90000; 
    double v_stop_eps = 0.3;          
    double v_max_mps = 60.0;          
};

class DrivePlant {
public:
    explicit DrivePlant(DriveParams p = {}, BatteryPlant* battery_plant = nullptr)
        : p_(p), battery_plant_(battery_plant) {}

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const DriveParams& params() const { return p_; }
    DriveParams& params() { return p_; }

private:
    DriveParams p_;
    BatteryPlant* battery_plant_;  // Reference to battery plant

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }

    double power_demand_kW_ = 0.0;
    double regen_power_kW_ = 0.0;
};

} // namespace plant
