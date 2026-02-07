// src/plant/wheel_subsystem/wheel_subsystem.hpp
#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include "plant/tyre_models/tyre_dugoff.hpp"

namespace plant {

struct WheelSubsystemParams {
    // Wheel dynamics parameters (single wheel)
    WheelDynamicsParams wheel;

    // Geometry
    double wheel_radius_m = 1.93;
    double wheelbase_m = 6.3;
    double track_m = 7.2;
    double cg_height_m = 3.2;

    // Surface friction (if you want quick runtime overrides)
    double mu_peak = 0.72;
    double mu_slide = 0.65;

    // Tire model parameters
    TyreDugoffParams tyre_params = {};

    // Enable dynamic Dugoff + wheel dynamics
    bool dynamic_mode_enabled = true;
};

class WheelSubsystem : public PhysicsSubsystem {
public:
    explicit WheelSubsystem(const WheelSubsystemParams& params = {});

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;

    const char* name() const override { return "Wheel"; }
    int priority() const override { return 105; }

    const WheelSubsystemParams& get_params() const { return p_; }
    void set_params(const WheelSubsystemParams& params);

private:
    WheelSubsystemParams p_;

    // Per-wheel rotational dynamics
    WheelDynamics wd_fl_;
    WheelDynamics wd_fr_;
    WheelDynamics wd_rl_;
    WheelDynamics wd_rr_;

    // Tire model
    TyreDugoff tyre_model_;

    // Helpers
    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }
};

} // namespace plant
