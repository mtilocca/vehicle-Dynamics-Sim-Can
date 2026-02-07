// src/plant/drive_subsystem/drive_plant.cpp
//
// DrivePlant Implementation - HYBRID MODEL WITH COAST REGEN
//
// CRITICAL FIX: Restored motor regen drag torque during coast (was removed!)

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
    const double v_abs = std::abs(v);

    // ========================================================================
    // STEP 0: Reset per-tick outputs (prevent stale data)
    // ========================================================================
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.motor_torque_nm = 0.0;

    // ========================================================================
    // STEP 1: Motor Torque Command with Coast Regen Drag
    // ========================================================================
    
    // Raw driver/requested torque (clamped)
    double motor_tq_cmd_raw = clamp(cmd.drive_torque_cmd_nm, 
                                    -p_.motor_torque_max_nm, 
                                    +p_.motor_torque_max_nm);

    // Default: if disabled, no positive propulsion allowed
    double motor_tq_cmd = enabled ? motor_tq_cmd_raw : 0.0;

    // Brake percentage
    const double brake_pct = clamp(cmd.brake_cmd_pct, 0.0, 100.0);

    // ========================================================================
    // COAST REGEN DETECTION (even when enabled=false!)
    // ========================================================================
    // CRITICAL: This was REMOVED but needs to be RESTORED!
    // The motor applies a velocity-proportional drag torque during coast
    // (lift-off) to simulate engine braking + regen charging.
    
    const bool coast_conditions =
        (std::abs(motor_tq_cmd_raw) < 1.0) &&  // no requested torque
        (brake_pct < 1.0) &&                   // not braking
        (v_abs > p_.v_stop_eps);               // moving

    bool regen_drag_active = false;
    
    if (coast_conditions) {
        // YAML-driven scaling (tune these parameters)
        const double regen_drag_max_frac  = 0.15;  // 15% of max torque
        const double regen_drag_v_ref_mps = 15.0;  // reach max at 15 m/s

        const double regen_drag_max_nm = regen_drag_max_frac * p_.motor_torque_max_nm;
        const double regen_drag_gain_nm_per_mps = 
            (regen_drag_v_ref_mps > 1e-6) ? (regen_drag_max_nm / regen_drag_v_ref_mps) : 0.0;

        const double T_regen_drag = 
            clamp(regen_drag_gain_nm_per_mps * v_abs, 0.0, regen_drag_max_nm);

        // Apply NEGATIVE torque (charging)
        // IMPORTANT: even in safe-mode, allow negative torque for coast regen
        motor_tq_cmd = -T_regen_drag;
        regen_drag_active = (T_regen_drag > 1e-3);
        
        LOG_DEBUG("[DrivePlant] Coast regen: v=%.2f m/s, T_drag=%.0f Nm", 
                  v, T_regen_drag);
    }

    // Publish motor torque EVERY tick
    s.motor_torque_nm = motor_tq_cmd;

    // ========================================================================
    // STEP 2: Wheel Torque from Motor (with power limiting)
    // ========================================================================
    
    double wheel_tq_total = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff;

    // Power limiting: τ_max = P_max / ω
    const double omega = std::max(v_abs, p_.v_stop_eps) / p_.wheel_radius_m;
    const double wheel_tq_power_max = p_.motor_power_max_w / omega;
    
    wheel_tq_total = clamp(wheel_tq_total, -wheel_tq_power_max, +wheel_tq_power_max);

    // ========================================================================
    // STEP 3: Brake Torque Distribution (40% front / 60% rear)
    // ========================================================================
    
    const double brake_tq_total = (brake_pct / 100.0) * p_.brake_torque_max_nm;
    const double brake_tq_front_axle = brake_tq_total * p_.brake_bias_front;
    const double brake_tq_rear_axle = brake_tq_total * (1.0 - p_.brake_bias_front);
    
    // Per-wheel brake torque (50/50 left-right)
    s.tau_brake_fl_nm = brake_tq_front_axle * 0.5;
    s.tau_brake_fr_nm = brake_tq_front_axle * 0.5;
    s.tau_brake_rl_nm = brake_tq_rear_axle * 0.5;
    s.tau_brake_rr_nm = brake_tq_rear_axle * 0.5;
    
    s.brake_force_kN = brake_tq_total / p_.wheel_radius_m / 1000.0;

    // ========================================================================
    // STEP 4: Drive Torque Distribution (RWD, 50/50 open diff)
    // ========================================================================
    
    s.tau_drive_fl_nm = 0.0;  // Front-left: non-driven
    s.tau_drive_fr_nm = 0.0;  // Front-right: non-driven
    s.tau_drive_rl_nm = wheel_tq_total * 0.5;  // Rear-left: 50%
    s.tau_drive_rr_nm = wheel_tq_total * 0.5;  // Rear-right: 50%

    // ========================================================================
    // STEP 5: Battery Energy Flow (with MOTOR REGEN DRAG)
    // ========================================================================
    
    // Power = Torque × Angular velocity (at motor shaft)
    const double motor_omega = omega * p_.gear_ratio;
    double power_demand_kW = motor_tq_cmd * motor_omega / 1000.0;

    // CRITICAL FIX: Step battery when enabled OR during coast regen
    if (battery_plant_ && (enabled || regen_drag_active)) {
        
        if (power_demand_kW > 0.01) {
            // Case 1: Driving (power consumption)
            double energy_J = power_demand_kW * dt_s * 1000.0;
            battery_plant_->consume_energy(energy_J, power_demand_kW);

            s.motor_power_kW = power_demand_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();

        } else if (power_demand_kW < -0.01) {
            // Case 2: Active regen (motor braking OR coast regen drag)
            double regen_power_kW = -power_demand_kW * p_.regen_eff_active;
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);

            s.motor_power_kW = power_demand_kW;
            s.regen_power_kW = regen_power_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();

            LOG_DEBUG("[DrivePlant] Regen: P_motor=%.2f kW, P_regen=%.2f kW, I=%.2f A", 
                      power_demand_kW, regen_power_kW, s.batt_i);
        }
        
        // Refresh battery telemetry EVERY tick we step the battery
        s.batt_i = battery_plant_->get_current();
        s.batt_v = battery_plant_->get_voltage();
    }

    // ========================================================================
    // STEP 6: Mode-Dependent Velocity/Wheel Speed Handling
    // ========================================================================
    
    if (s.dynamic_model_enabled) {
        // DYNAMIC MODE: WheelSubsystem + VehicleSubsystem handle integration
        // We've already populated tau_drive_* and tau_brake_*
        // Nothing more to do!
        
    } else {
        // KINEMATIC MODE: Classic V1 behavior (backward compatibility)
        
        // Resistive forces
        const double F_drag = p_.drag_c * v * std::abs(v);
        const double F_roll = p_.roll_c * static_cast<double>(sgn(v));
        const double F_res = F_drag + F_roll;
        
        // Drive force from torque
        const double Fx = wheel_tq_total / p_.wheel_radius_m;
        
        // Brake force (opposes motion)
        const double F_brake = brake_tq_total / p_.wheel_radius_m * static_cast<double>(sgn(v));
        
        // Net force and acceleration
        const double F_net = Fx - F_brake - F_res;
        const double a = F_net / p_.mass_kg;

        // Velocity integration
        double v_next = v + a * dt_s;
        v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

        // Zero-crossing detection
        if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
            if (std::abs(v_next) < 0.05)
                v_next = 0.0;
        }

        s.a_long_mps2 = a;
        s.v_mps = v_next;

        // Derive wheel speeds from vehicle velocity (no slip)
        const double wheel_omega = s.v_mps / p_.wheel_radius_m;
        const double wheel_rps = wheel_omega / (2.0 * M_PI);
        
        s.omega_fl_radps = wheel_omega;
        s.omega_fr_radps = wheel_omega;
        s.omega_rl_radps = wheel_omega;
        s.omega_rr_radps = wheel_omega;
        
        s.wheel_fl_rps = wheel_rps;
        s.wheel_fr_rps = wheel_rps;
        s.wheel_rl_rps = wheel_rps;
        s.wheel_rr_rps = wheel_rps;
    }

    static int cnt = 0;
    if (cnt++ % 100 == 0) {
        LOG_DEBUG("[DrivePlant] v=%.2f m/s, τ_motor=%.0f Nm, τ_wheel=%.0f Nm, "
                  "coast_regen=%d, P_motor=%.2f kW, I=%.2f A",
                  v, motor_tq_cmd, wheel_tq_total, 
                  (int)regen_drag_active, s.motor_power_kW, s.batt_i);
    }
}

} // namespace plant