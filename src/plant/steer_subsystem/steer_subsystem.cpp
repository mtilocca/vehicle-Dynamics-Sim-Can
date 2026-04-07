// src/plant/steer_subsystem/steer_subsystem.cpp

#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "utils/logging.hpp"
#ifdef __ZEPHYR__
LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);
#endif

namespace plant {

SteerSubsystem::SteerSubsystem(const SteerParams& params)
    : steer_(params)
{
}

void SteerSubsystem::initialize(PlantState& s) {
    LOG_INFO("[SteerSubsystem] Initializing:");
    LOG_INFO("  max_angle=%.1f°, rate=%.1f°/s",
             steer_.params().delta_max_deg, 
             steer_.params().steer_rate_dps);
    LOG_INFO("  wheelbase=%.2f m, track=%.2f m, ackermann=%.0f%%",
             steer_.params().wheelbase_m, 
             steer_.params().track_width_m,
             steer_.params().ackermann_pct * 100.0);

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
    steer_.step(s, cmd, dt);
    
    LOG_DEBUG("[SteerSubsystem] virtual=%.2f°, FL=%.2f°, FR=%.2f°",
              s.steer_virtual_rad * 180.0 / 3.14159265,
              s.delta_fl_rad * 180.0 / 3.14159265,
              s.delta_fr_rad * 180.0 / 3.14159265);
}

void SteerSubsystem::set_params(const SteerParams& params) {
    steer_.params() = params;
    LOG_INFO("[SteerSubsystem] Parameters updated: ackermann=%.0f%%",
             params.ackermann_pct * 100.0);
}

} // namespace plant