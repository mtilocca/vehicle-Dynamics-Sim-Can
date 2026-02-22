// src/plant/drive_subsystem/drive_plant.hpp
//
// DrivePlant - Simplified powertrain + linear tire model
//
// Physics pipeline:
//   1. Motor torque → traction force (gear ratio, efficiency, power limit, friction limit)
//   2. Brake torque → braking force (front/rear bias, L/R 50-50)
//   3. Static Fz per wheel (weight distribution from geometry)
//   4. Slip angles from body velocities and steering (linear bicycle model)
//   5. Lateral forces: Fy = -Cy × alpha  (opposes slip, RWD: rear only driven)
//   6. Wheel speeds derived from v (no-slip)
//
// Outputs written to PlantState:
//   Fx_*/Fy_*/Fz_* per wheel  → consumed by VehicleSubsystem (3-DOF)
//   wheel_*_rps               → CAN TX
//   motor_torque_nm, brake_force_kN

#pragma once

#include <algorithm>
#include <cmath>
#include "plant/plant_main/plant_state.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct DriveParams {
    // Vehicle mass
    double mass_kg = 218000.0;

    // Wheel geometry
    double wheel_radius_m = 1.93;

    // Resistive forces
    double drag_c = 2.5;           // Aerodynamic drag [N/(m/s)²]
    double roll_c = 1500.0;        // Rolling resistance [N]

    // Motor limits
    double motor_torque_max_nm = 9500.0;
    double motor_power_max_w   = 2800000.0;

    // Brake system
    double brake_torque_max_nm = 80000.0;
    double brake_bias_front    = 0.40;   // Front axle brake proportion

    // Drivetrain
    double gear_ratio      = 25.0;
    double drivetrain_eff  = 0.92;

    // Speed limits
    double v_stop_eps = 0.3;
    double v_max_mps  = 60.0;
    double v_kinematic_blend_mps = 3.0;  // Low-speed kinematic blend threshold [m/s]

    // Simple friction limit for traction
    double mu_surface = 0.72;

    // Linear cornering stiffness [N/rad] per axle (total, split 50/50 per wheel)
    double Cy_front_Npm = 2500000.0;  // front axle total
    double Cy_rear_Npm  = 2000000.0;  // rear axle total

    // Tire relaxation (first-order lag on lateral forces)
    double tire_relax_tau_s = 0.3;   // [s]

    // CG geometry (needed for weight distribution and slip angles)
    double cg_to_front_m = 2.52;   // lf: CG → front axle [m]
    double wheelbase_m   = 6.30;   // L = lf + lr [m]
};

// ============================================================================
// DrivePlant - Simplified powertrain + linear tire
// ============================================================================

class DrivePlant {
public:
    explicit DrivePlant(DriveParams p = {}) : p_(p) {}

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const DriveParams& params() const { return p_; }
    DriveParams& params() { return p_; }

private:
    DriveParams p_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }
};

} // namespace plant
