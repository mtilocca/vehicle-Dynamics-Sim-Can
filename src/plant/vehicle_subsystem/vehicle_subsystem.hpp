// src/plant/vehicle_subsystem/vehicle_subsystem.hpp
//
// VehicleSubsystem - Longitudinal and Yaw Dynamics Integration
//
// Priority: 110 (After WheelSubsystem)
// Executes AFTER WheelSubsystem computes tire forces

#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include <cmath>
#include <algorithm>

namespace plant {

struct VehicleParams {
    // Vehicle Mass and Geometry (3-DOF: longitudinal, lateral, yaw)
    double mass_kg = 218000.0;          // Vehicle mass [kg]
    double wheelbase_m = 6.3;           // Wheelbase L [m]
    double track_m = 7.2;               // Track width [m]
    double cg_to_front_m = 2.52;        // CG to front axle distance [m]
    double yaw_inertia_kgm2 = 8.5e6;    // Yaw moment of inertia Iz [kg·m²]

    // Resistive Forces
    double drag_c = 2.5;                // Aerodynamic drag coefficient [N/(m/s)²]
    double roll_c = 1500.0;             // Rolling resistance [N]

    // Speed Limits
    double v_stop_eps    = 0.3;         // Standstill threshold [m/s]
    double v_max_mps     = 16.667;      // Maximum forward velocity [m/s] (60 km/h)
    double v_max_rev_mps = 5.556;       // Maximum reverse velocity [m/s] (20 km/h)
    double v_lat_max_mps = 8.0;         // Lateral velocity clamp (prevents divergence)

    // Reverse / intent handling
    bool allow_reverse = true;          // Allow vehicle to drive in reverse
    double torque_dir_deadband_nm = 50.0; // Below this, treat torque as "no intent"
};

class VehicleSubsystem : public PhysicsSubsystem {
public:
    explicit VehicleSubsystem(const VehicleParams& params = {});

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;

    const char* name() const override { return "Vehicle"; }
    int priority() const override { return 110; }

    const VehicleParams& get_params() const { return p_; }
    void set_params(const VehicleParams& params);

private:
    VehicleParams p_;

    // Direction latch: +1 forward, -1 reverse, 0 unknown/neutral
    int dir_latch_ = 0;

    void update_direction_latch(const PlantState& s, const sim::ActuatorCmd& cmd);

    void compute_resistive_forces(
        double v_mps,
        int dir_ref,
        double& Fdrag_out,
        double& Froll_out
    ) const;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) {
        return (x > 0.0) - (x < 0.0);
    }
};

} // namespace plant
