#include "steer_plant.hpp"

#include <algorithm>
#include <cmath>

#include "vehicle_bicycle_ackermann.hpp"
#include "sim/actuator_cmd.hpp"

namespace plant {

static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

void SteerPlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    // 1) Read command (deg) -> desired virtual steer (rad)
    const double delta_cmd_rad = deg2rad(cmd.steer_cmd_deg);

    // 2) Clamp desired steer
    const double delta_max_rad = deg2rad(p_.delta_max_deg);
    const double delta_des_rad = clamp(delta_cmd_rad, -delta_max_rad, +delta_max_rad);

    // 3) Rate-limit current steer toward desired steer
    const double max_step = deg2rad(p_.steer_rate_dps) * dt_s;
    const double err = delta_des_rad - s.steer_virtual_rad;
    const double delta_step = clamp(err, -max_step, +max_step);

    s.steer_virtual_rad = clamp(s.steer_virtual_rad + delta_step, -delta_max_rad, +delta_max_rad);

    // 4) Ackermann mapping to physical wheel angles
    BicycleAckermannParams ap{};
    ap.L_m = p_.wheelbase_m;
    ap.W_m = p_.track_width_m;
    ap.delta_max_rad = delta_max_rad;

    VehicleBicycleAckermann::ackermann_map(
        s.steer_virtual_rad,
        ap,
        s.delta_fl_rad,
        s.delta_fr_rad,
        nullptr
    );
}

} // namespace plant
