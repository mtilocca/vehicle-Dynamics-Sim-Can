// src/plant/wheel_subsystem/wheel_subsystem.hpp
#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include "plant/tyre_models/tyre_dugoff.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace plant {

/**
 * WheelSubsystemParams - Configuration for wheel dynamics subsystem
 *
 * Contains geometry and physics parameters for 4-wheel dynamics.
 */
struct WheelSubsystemParams {
    // Individual wheel parameters (same for all wheels by default)
    WheelDynamicsParams wheel{};

    // Vehicle geometry
    double wheelbase_m = 6.30;       // Front-to-rear axle distance
    double track_width_m = 7.20;     // Left-to-right wheel distance
    double cg_height_m = 3.20;       // Center of gravity height (load transfer)
    double cg_to_front_m = 2.52;     // CG -> front axle distance
    double cg_to_rear_m  = 3.78;     // CG -> rear axle distance

    // Vehicle mass (for load transfer calculation)
    double mass_kg = 218000.0;

    // Surface friction (passed into TyreDugoffParams)
    double mu_peak  = 0.72;
    double mu_slide = 0.65;

    // Static axle load split sanity (optional)
    bool enable_load_transfer = true;
};

/**
 * WheelSubsystem - Wheel angular dynamics + Dugoff tire force model
 *
 * Priority: 105 (After DriveSubsystem, before VehicleSubsystem)
 *
 * Responsibilities:
 * - Integrate wheel angular velocity omega for each wheel
 * - Compute slip ratios using robust direction handling near 0 speed
 * - Call Dugoff tire model to compute Fx/Fy + lambda
 * - Compute normal loads with longitudinal load transfer
 * - Write forces + slip + lambda into PlantState
 */
class WheelSubsystem : public PhysicsSubsystem {
public:
    explicit WheelSubsystem(const WheelSubsystemParams& params = WheelSubsystemParams());

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;

    const char* name() const override { return "Wheel"; }
    int priority() const override { return 105; }

    const WheelSubsystemParams& get_params() const { return p_; }
    void set_params(const WheelSubsystemParams& params);

private:
    WheelSubsystemParams p_;

    // Wheel dynamics per-corner
    WheelDynamics w_fl_;
    WheelDynamics w_fr_;
    WheelDynamics w_rl_;
    WheelDynamics w_rr_;

    // Single tire model instance (same params for all wheels)
    TyreDugoff tyre_model_;
    TyreDugoffParams tyre_params_;

private:
    void apply_tyre_params_();

    void compute_normal_loads_(PlantState& s, double a_long, double& Fz_fl, double& Fz_fr, double& Fz_rl, double& Fz_rr) const;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }
};

} // namespace plant
