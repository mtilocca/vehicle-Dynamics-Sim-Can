#include "steer_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include "plant/vehicle_bicycle_ackermann.hpp"

#include <algorithm>
#include <cmath>

namespace plant {

void SteerPlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    if (dt_s <= 0.0) return;

    const bool enabled = cmd.system_enable;

    const double delta_cmd_rad = enabled ? deg2rad(cmd.steer_cmd_deg) : 0.0;
    const double base_delta_max_rad = deg2rad(p_.delta_max_deg);

    // ------------------------------------------------------------
    // Speed-dependent steering reduction (simple understeer)
    // ------------------------------------------------------------
    double ratio = 1.0;
    if (s.v_mps > p_.v_steer_limit_start_mps) {
        const double v0 = p_.v_steer_limit_start_mps;
        const double v1 = std::max(p_.v_steer_limit_end_mps, v0 + 1e-6);
        const double a =
            clamp((s.v_mps - v0) / (v1 - v0), 0.0, 1.0);
        ratio = (1.0 - a) + a * p_.steer_limit_ratio_highv;
    }

    const double delta_max_rad = base_delta_max_rad * ratio;

    const double delta_des_rad =
        clamp(delta_cmd_rad, -delta_max_rad, +delta_max_rad);

    // ------------------------------------------------------------
    // Rate limiting
    // ------------------------------------------------------------
    const double max_step = deg2rad(p_.steer_rate_dps) * dt_s;
    const double delta_prev = s.steer_virtual_rad;

    const double delta_next =
        clamp(delta_prev +
              clamp(delta_des_rad - delta_prev, -max_step, +max_step),
              -delta_max_rad, +delta_max_rad);

    s.steer_virtual_rad = delta_next;
    s.steer_rate_radps = (delta_next - delta_prev) / dt_s;

    // ------------------------------------------------------------
    // Ackermann mapping
    // ------------------------------------------------------------
    BicycleAckermannParams ap{};
    ap.L_m = p_.wheelbase_m;
    ap.W_m = p_.track_width_m;
    ap.delta_max_rad = base_delta_max_rad; // physical hard limit

    VehicleBicycleAckermann::ackermann_map(
        delta_next,
        ap,
        s.delta_fl_rad,
        s.delta_fr_rad,
        nullptr
    );
}

} // namespace plant
