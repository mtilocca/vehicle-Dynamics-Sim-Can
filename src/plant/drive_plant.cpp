// src/plant/drive_plant.cpp
#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"

#include <cmath>

namespace plant {

void DrivePlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    if (dt_s <= 0.0) return;

    // If disabled, coast down with resistances only (V1 policy)
    const bool enabled = cmd.system_enable;

    const double v = s.v_mps;

    // --- Resistive forces (always applied)
    // Drag opposes motion naturally with v*|v|
    const double F_drag = p_.drag_c * v * std::abs(v);

    // Rolling resistance opposes motion (use sign)
    const double F_roll = p_.roll_c * static_cast<double>(sgn(v));

    const double F_res = F_drag + F_roll;

    // --- Wheel force from motor + brake
    double Fx = 0.0;

    // Brake torque always opposes motion (wheel angular speed)
    const double brake_pct = clamp(cmd.brake_cmd_pct, 0.0, 100.0);
    const double brake_tq_mag = (brake_pct / 100.0) * p_.brake_torque_max_nm;
    const double brake_tq = brake_tq_mag * static_cast<double>(sgn(v));

    if (enabled) {
        // Motor command torque at motor shaft
        double motor_tq_cmd = clamp(cmd.drive_torque_cmd_nm,
                                    -p_.motor_torque_max_nm,
                                    +p_.motor_torque_max_nm);

        // Convert to wheel torque via gear ratio + efficiency
        double wheel_tq_from_motor = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff;

        // Power limiting: T_wheel_max = P_max * r / max(|v|, eps)
        const double denom_v = std::max(std::abs(v), p_.v_stop_eps);
        const double wheel_tq_power_max = (p_.motor_power_max_w * p_.wheel_radius_m) / denom_v;

        wheel_tq_from_motor = clamp(wheel_tq_from_motor,
                                    -wheel_tq_power_max,
                                    +wheel_tq_power_max);

        // Net wheel torque at tire (brake opposes motion)
        const double wheel_tq = wheel_tq_from_motor - brake_tq;

        Fx = wheel_tq / p_.wheel_radius_m;
    } else {
        // Disabled: no motor; allow braking to still apply if you want.
        // V1: treat brake as still active when disabled.
        Fx = (-brake_tq) / p_.wheel_radius_m;
    }

    // --- Net force and acceleration
    const double F_net = Fx - F_res;
    const double a = F_net / p_.mass_kg;

    // Integrate speed
    double v_next = v + a * dt_s;

    // Hard speed clamp
    v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

    // If we cross through zero, snap to zero to avoid tiny oscillations
    if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
        if (std::abs(v_next) < 0.05) v_next = 0.0;
    }

    s.a_long_mps2 = a;
    s.v_mps = v_next;
}

} // namespace plant
