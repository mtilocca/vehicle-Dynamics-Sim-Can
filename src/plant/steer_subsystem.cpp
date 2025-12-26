// src/plant/steer_subsystem.cpp
#include "plant/steer_subsystem.hpp"
#include "utils/logging.hpp"

namespace plant {

SteerSubsystem::SteerSubsystem(const SteerParams& params)
    : steer_(params)
{
}

void SteerSubsystem::initialize(PlantState& s) {
    LOG_INFO("[SteerSubsystem] Initializing: max_angle=%.1f deg, rate=%.1f deg/s, wheelbase=%.2f m",
             steer_.params().delta_max_deg,
             steer_.params().steer_rate_dps,
             steer_.params().wheelbase_m);

    // Set initial steering to zero
    s.steer_virtual_rad = 0.0;
    s.steer_rate_radps = 0.0;
    s.delta_fl_rad = 0.0;
    s.delta_fr_rad = 0.0;
}

void SteerSubsystem::reset(PlantState& s) {
    LOG_INFO("[SteerSubsystem] Resetting to zero steering angle");
    
    s.steer_virtual_rad = 0.0;
    s.steer_rate_radps = 0.0;
    s.delta_fl_rad = 0.0;
    s.delta_fr_rad = 0.0;
}

void SteerSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Delegate to existing SteerPlant implementation
    steer_.step(s, cmd, dt);
    
    LOG_DEBUG("[SteerSubsystem] virtual=%.2f deg, FL=%.2f deg, FR=%.2f deg",
              s.steer_virtual_rad * 180.0 / 3.14159265,
              s.delta_fl_rad * 180.0 / 3.14159265,
              s.delta_fr_rad * 180.0 / 3.14159265);
}

void SteerSubsystem::set_params(const SteerParams& params) {
    steer_.params() = params;
}

} // namespace plant
