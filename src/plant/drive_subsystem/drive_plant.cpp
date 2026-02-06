// src/plant/drive_subsystem/drive_plant.cpp
#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

void DrivePlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s)
{
    if (dt_s <= 0.0)
        return;

    const bool enabled = cmd.system_enable;
    const double v = s.v_mps;

    // ---------------------------------------------------------------------
    // Reset per-tick published battery outputs to avoid stale plots
    // ---------------------------------------------------------------------
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = 0.0;
    s.batt_i = 0.0; // if we don't step battery this tick, show neutral
    // Keep s.batt_v as last known unless you want to zero it too
    // ---------------------------------------------------------------------

    // --- Resistive forces (always applied)
    const double F_drag = p_.drag_c * v * std::abs(v);
    const double F_roll = p_.roll_c * static_cast<double>(sgn(v));
    const double F_res  = F_drag + F_roll;

    double Fx = 0.0;

    // --- Brake force calculation based on brake percentage
    const double brake_pct    = clamp(cmd.brake_cmd_pct, 0.0, 100.0);
    const double brake_tq_mag = (brake_pct / 100.0) * p_.brake_torque_max_nm;
    const double brake_tq     = brake_tq_mag * static_cast<double>(sgn(v));

    // Calculate brake force (in kN)
    const double brake_force_kN = brake_tq / p_.wheel_radius_m / 1000.0;
    s.brake_force_kN = brake_force_kN;

    static int tqctn = 0;
    if (tqctn++ % 100 == 0) {
        LOG_DEBUG("[DrivePlant] enabled=%d v=%.2f m/s, motor_cmd=%.0f Nm, brake=%.1f%%",
                 (int)enabled, v, cmd.drive_torque_cmd_nm, brake_pct);
    }

    // =====================================================================
    // Motor torque command (safe-mode gating + auto coast regen drag)
    // =====================================================================

    // Raw driver/requested torque (clamped)
    const double motor_tq_cmd_raw =
        clamp(cmd.drive_torque_cmd_nm, -p_.motor_torque_max_nm, +p_.motor_torque_max_nm);

    // Default: if disabled, no positive propulsion torque allowed
    double motor_tq_cmd = enabled ? motor_tq_cmd_raw : 0.0;

    const double v_abs = std::abs(v);

    // "Lift-off"/coast detection SHOULD NOT require enabled=1, because your sim
    // enters safe-mode (enabled=0) during CAN timeout and you still want visible
    // coast regen while rolling.
    const bool coast_conditions =
        (std::abs(motor_tq_cmd_raw) < 1.0) &&  // no requested torque
        (brake_pct < 1.0) &&                   // not braking
        (v_abs > p_.v_stop_eps);               // moving

    // Auto "regen drag torque" on coasting (allowed even if enabled=0)
    bool regen_drag_active = false;
    if (coast_conditions) {
        // YAML-driven scaling (Option A-style, but derived from YAML max torque)
        // Tune these 2 knobs:
        const double regen_drag_max_frac  = 0.15; // 15% of max torque (tune)
        const double regen_drag_v_ref_mps = 15.0; // reach cap at 15 m/s (tune)

        const double regen_drag_max_nm =
            regen_drag_max_frac * p_.motor_torque_max_nm;

        const double regen_drag_gain_nm_per_mps =
            (regen_drag_v_ref_mps > 1e-6) ? (regen_drag_max_nm / regen_drag_v_ref_mps) : 0.0;

        const double T_regen_drag =
            clamp(regen_drag_gain_nm_per_mps * v_abs, 0.0, regen_drag_max_nm);

        // Apply negative torque => charging (if your battery uses signed power)
        // IMPORTANT: even in safe-mode, allow *negative* torque for coast regen,
        // but never allow positive propulsion when disabled.
        motor_tq_cmd = -T_regen_drag;
        regen_drag_active = (T_regen_drag > 1e-3);
    }

    // Publish motor torque EVERY tick (prevents stale torque)
    s.motor_torque_nm = motor_tq_cmd;

    // Wheel torque from motor (apply drivetrain efficiency)
    double wheel_tq_from_motor = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff;

    // Power limiting
    const double denom_v = std::max(std::abs(v), p_.v_stop_eps);
    const double wheel_tq_power_max = (p_.motor_power_max_w * p_.wheel_radius_m) / denom_v;
    wheel_tq_from_motor = clamp(wheel_tq_from_motor, -wheel_tq_power_max, +wheel_tq_power_max);

    // Net wheel torque (motor + brake)
    const double wheel_tq = wheel_tq_from_motor - brake_tq;

    // ========================================================================
    // TRACTION LIMITING (dynamic model)
    // ========================================================================
    const double Fx_demanded = wheel_tq / p_.wheel_radius_m;

    if (s.dynamic_model_enabled) {
        const double Fx_available = std::abs(s.Fx_rl) + std::abs(s.Fx_rr);
        if (std::abs(Fx_demanded) > Fx_available && Fx_available > 0.0) {
            Fx = std::copysign(Fx_available, Fx_demanded);
            LOG_DEBUG("[DrivePlant] TRACTION LIMITED: demanded=%.0f N, available=%.0f N",
                      Fx_demanded, Fx_available);
        } else {
            Fx = Fx_demanded;
        }
    } else {
        Fx = Fx_demanded;
    }

    // --- Power demand calculation (Torque * Angular Velocity)
    const double angular_velocity = v / p_.wheel_radius_m;                           // rad/s
    const double power_demand_kW  = wheel_tq_from_motor * angular_velocity / 1000.0; // kW

    LOG_DEBUG("[DrivePlant] wheel_tq=%.2f Nm, omega=%.2f rad/s, P_demand=%.2f kW",
              wheel_tq_from_motor, angular_velocity, power_demand_kW);

    // =====================================================================
    // Battery update (canonical)
    // =====================================================================
    // Key fix: step battery not only when enabled, but also when coast-regen is active.
    if (battery_plant_ && (enabled || regen_drag_active)) {
        battery_plant_->step(power_demand_kW, brake_force_kN, dt_s);

        // Signed power (+ discharge, - charge)
        s.motor_power_kW = power_demand_kW;

        // For plotting convenience (positive magnitude)
        if (power_demand_kW < 0.0) {
            s.regen_power_kW = (-power_demand_kW);
        }

        // Refresh battery telemetry EVERY tick we step the battery
        s.batt_i = battery_plant_->get_current();  // should go negative in coast regen
        s.batt_v = battery_plant_->get_voltage();
    }

    // ====================================================================
    // VELOCITY INTEGRATION (kinematic mode only)
    // ====================================================================
    // Don't gate this on enabled. Even in safe-mode you want the vehicle to
    // slow down (at least from resistive forces), and if regen_drag_active it
    // should slow down more.
    if (!s.dynamic_model_enabled) {
        const double F_net = Fx - F_res;
        const double a     = F_net / p_.mass_kg;

        double v_next = v + a * dt_s;
        v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

        if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
            if (std::abs(v_next) < 0.05)
                v_next = 0.0;
        }

        s.a_long_mps2 = a;
        s.v_mps       = v_next;
    }

    static int cnt1 = 0;
    if (cnt1++ % 100 == 0) {
        LOG_DEBUG("[DrivePlant] enabled=%d regen_drag=%d tq_raw=%.1f tq_applied=%.1f P=%.2f kW I=%.2f A",
                 (int)enabled, (int)regen_drag_active, motor_tq_cmd_raw, motor_tq_cmd, s.motor_power_kW, s.batt_i);
    }

    // Wheel speeds
    const double wheel_rps = s.v_mps / p_.wheel_radius_m;
    s.wheel_fl_rps = wheel_rps;
    s.wheel_fr_rps = wheel_rps;
    s.wheel_rl_rps = wheel_rps;
    s.wheel_rr_rps = wheel_rps;
}

} // namespace plant
