// src/plant/drive_plant.cpp
// ULTIMATE FIX: Re-read battery state AFTER storing regen energy
#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static inline int sgn(double x) {
    return (x > 0.0) - (x < 0.0);
}

void DrivePlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    if (dt_s <= 0.0) return;

    // Extract commands
    const bool enabled = cmd.system_enable;
    const double motor_tq_cmd = cmd.drive_torque_cmd_nm;
    const double brake_pct = cmd.brake_cmd_pct;

    // Current velocity (rear axle frame)
    const double v = s.v_mps;

    // === Longitudinal Dynamics ===
    // Wheel normal force ~ assume we're on level ground
    // Aerodynamic drag: F_drag = - drag_c * v^2 * sign(v)
    const double F_drag = -p_.drag_c * v * std::abs(v);
    
    // Rolling resistance (simplified Coulomb model)
    const double F_roll = -sgn(v) * p_.roll_c;
    
    // Total resistance force
    const double F_res = F_drag + F_roll;

    // Motor torque at the wheel (after gear ratio)
    const double motor_tq_wheel = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff;
    
    // Wheel-to-road force from motor
    double Fx = motor_tq_wheel / p_.wheel_radius_m;

    // Brake torque at the wheel
    const double brake_tq = clamp(brake_pct / 100.0, 0.0, 1.0) * p_.brake_torque_max_nm;
    const double brake_force_kN = brake_tq / p_.wheel_radius_m / 1000.0; // Convert to kN

    // Calculate power demand for battery
    const double motor_speed_radps = std::abs(v) / p_.wheel_radius_m;
    const double motor_shaft_speed_radps = motor_speed_radps * p_.gear_ratio;
    const double power_demand_kW = (motor_tq_cmd * motor_shaft_speed_radps) / 1000.0;

    LOG_DEBUG("[DrivePlant] v=%.2f m/s, motor_tq=%.1f Nm, brake=%.1f%%, power=%.2f kW, vel=%.2f, power_kW=%.2f",
              v, motor_tq_cmd, brake_pct, power_demand_kW, v, power_demand_kW);

    // Initialize battery state fields
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = brake_force_kN;

    // --- Update battery and apply forces
    if (enabled) {
        // Update battery with power demand
        if (battery_plant_) {
            battery_plant_->step(power_demand_kW, brake_force_kN, dt_s);
            
            // Read back battery state into PlantState (FIRST READ)
            s.batt_soc_pct = battery_plant_->get_soc() * 100.0;  // Convert 0-1 to 0-100
            s.batt_v = battery_plant_->get_voltage();
            s.batt_i = battery_plant_->get_current();
            s.motor_power_kW = power_demand_kW;
            
            LOG_DEBUG("[DrivePlant] Battery: SOC=%.1f%%, V=%.1f V, I=%.2f A", 
                      s.batt_soc_pct, s.batt_v, s.batt_i);
        }
    } else {
        // System disabled - no motor power, apply only brake force
        Fx = (-brake_tq) / p_.wheel_radius_m;
    }

    // ============================================================================
    // BUG FIX #2 & #3: Regenerative braking (active braking + coasting)
    // ============================================================================
    bool regen_active = false;
    double regen_power_kW = 0.0;

    // Case 1: Active braking with brake pedal
    if (brake_pct > 0.0 && std::abs(v) > p_.v_stop_eps && motor_tq_cmd == 0.0) {
        const double regen_eff = 0.7;  // 70% efficiency for active braking
        regen_power_kW = brake_force_kN * std::abs(v) * regen_eff;  // kW
        regen_active = true;
        
        LOG_DEBUG("[DrivePlant] Active Regen: P_regen=%.2f kW, brake_force=%.2f kN", 
                  regen_power_kW, brake_force_kN);
    }
    // Case 2: Coasting regenerative braking
    else if (motor_tq_cmd == 0.0 && std::abs(v) > p_.v_stop_eps && brake_pct == 0.0) {
        // During coasting, the drivetrain couples wheels to motor
        // The resistance forces (drag + rolling) cause the motor to spin
        // which generates electricity
        const double resistance_power_kW = std::abs(F_res * v) / 1000.0;
        const double coasting_regen_eff = 0.3;  // 30% efficiency for passive regen
        regen_power_kW = resistance_power_kW * coasting_regen_eff;
        regen_active = true;
        
        LOG_DEBUG("[DrivePlant] Coasting Regen: P_regen=%.2f kW, F_res=%.2f N", 
                  regen_power_kW, F_res);
    }

    // Store regen energy if active
    if (regen_active && battery_plant_) {
        battery_plant_->store_energy(regen_power_kW * dt_s * 1000.0);  // Convert kW*s to J
        s.regen_power_kW = regen_power_kW;
        
        // ========================================================================
        // CRITICAL FIX: Re-read battery state AFTER storing regen energy
        // ========================================================================
        s.batt_soc_pct = battery_plant_->get_soc() * 100.0;
        s.batt_v = battery_plant_->get_voltage();
        s.batt_i = battery_plant_->get_current();  // ← NOW this will be NEGATIVE! ✅
        
        LOG_DEBUG("[DrivePlant] After Regen: SOC=%.1f%%, V=%.1f V, I=%.2f A", 
                  s.batt_soc_pct, s.batt_v, s.batt_i);
    }

    // --- Net force and acceleration
    const double F_net = Fx - F_res;
    const double a = F_net / p_.mass_kg;

    double v_next = v + a * dt_s;
    v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

    // Snap to zero if speed crosses zero to avoid tiny oscillations
    if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
        if (std::abs(v_next) < 0.05) v_next = 0.0;
    }

    s.a_long_mps2 = a;
    s.v_mps = v_next;
    s.motor_torque_nm = motor_tq_cmd;
}

} // namespace plant