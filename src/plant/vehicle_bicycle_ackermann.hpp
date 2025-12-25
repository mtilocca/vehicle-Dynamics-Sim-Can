// src/plant/vehicle_bicycle_ackermann.hpp
#pragma once

#include <cstdint>

namespace plant {

// Parameters for Ackermann-constrained kinematic bicycle
struct BicycleAckermannParams {
    double L_m = 2.8;     // wheelbase
    double W_m = 1.6;     // track width (front)
    double delta_max_rad = 35.0 * 3.141592653589793 / 180.0; // steering limit
    double eps_rad = 1e-6;       // near-zero steer threshold
    double tiny_m = 1e-6;        // avoid singularities
};

struct BicycleState2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

struct BicycleStepResult {
    BicycleState2D next;
    double yaw_rate_rps = 0.0;
    double R_m = 0.0;            // signed radius (if delta!=0), else +inf approx
    double delta_fl_rad = 0.0;
    double delta_fr_rad = 0.0;
};

class VehicleBicycleAckermann {
public:
    // Stateless step: integrates pose given v and virtual steer delta (rear-axle reference).
    // Also returns Ackermann physical front wheel angles.
    static BicycleStepResult step(
        const BicycleState2D& s,
        double v_mps,
        double delta_virtual_rad,
        const BicycleAckermannParams& p,
        double dt_s
    );

    // Stateless mapping only (no integration)
    static void ackermann_map(
        double delta_virtual_rad,
        const BicycleAckermannParams& p,
        double& delta_fl_rad_out,
        double& delta_fr_rad_out,
        double* R_m_out = nullptr
    );

private:
    static double clamp(double v, double lo, double hi);
    static int sign(double v);
};

} // namespace plant
