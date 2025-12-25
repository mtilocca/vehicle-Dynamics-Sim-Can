#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"


#include <algorithm>
#include <cmath>

#include "sim/actuator_cmd.hpp"

namespace plant {

static inline double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

void DrivePlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    // Interpret commands
    const double motor_tq = clamp(cmd.drive_torque_cmd_nm, -p_.motor_torque_max_nm, +p_.motor_torque_max_nm);

    // brake_cmd_pct is 0..100 (per CAN map). Convert to brake torque.
    const double brake_pct = clamp(cmd.brake_cmd_pct, 0.0, 100.0);
    const double brake_tq  = (brake_pct / 100.0) * p_.brake_torque_max_nm; // always resistive

    // Net wheel torque
    const double wheel_tq = motor_tq - brake_tq;

    // Drive/brake force at contact patch (simple)
    double Fx = wheel_tq / p_.wheel_radius_m; // N

    // Resistive forces (simple)
    const double v = s.v_mps;
    const double F_drag = p_.drag_c * v * v;
    const double F_roll = p_.roll_c;

    // Oppose motion (V1 assumes v>=0)
    const double F_resist = (v >= 0.0) ? (F_drag + F_roll) : -(F_drag + F_roll);

    // Acceleration
    const double a = (Fx - F_resist) / p_.mass_kg;

    // Integrate speed (do not allow negative speed in V1)
    s.v_mps = std::max(0.0, s.v_mps + a * dt_s);

    // Optional: derive wheel speeds (very simple, no slip)
    const double wheel_rps = (p_.wheel_radius_m > 1e-9) ? (s.v_mps / p_.wheel_radius_m) / (2.0 * M_PI) : 0.0;
    s.wheel_fl_rps = wheel_rps;
    s.wheel_fr_rps = wheel_rps;
    s.wheel_rl_rps = wheel_rps;
    s.wheel_rr_rps = wheel_rps;
}

} // namespace plant
