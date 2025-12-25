#pragma once

#include <cmath>
#include "battery_plant.hpp"  // Include BatteryPlant class for energy management

namespace plant {

struct BicycleAckermannParams {
    double L_m = 2.8;
    double W_m = 1.6;
    double delta_max_rad = 0.6;

    // --- NEW realism knob
    double mu_lat = 0.9;   // lateral friction coefficient
    double g = 9.81;
};

struct BicycleState2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
    double speed_mps = 0.0;  // Add speed for energy calculation
};

struct BicycleStepResult {
    BicycleState2D next;
    double yaw_rate_rps = 0.0;
    double delta_fl_rad = 0.0;
    double delta_fr_rad = 0.0;
};

class VehicleBicycleAckermann {
public:
    static void ackermann_map(
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double& delta_fl_rad,
        double& delta_fr_rad,
        double* curvature_out = nullptr
    );

    // ✅ REQUIRED by plant_model.cpp
    static BicycleStepResult step(
        const BicycleState2D& s,
        double v_mps,
        double steer_virtual_rad,
        const BicycleAckermannParams& p,
        double dt_s,
        BatteryPlant& battery_plant  // Add reference to BatteryPlant
    );
};

} // namespace plant
