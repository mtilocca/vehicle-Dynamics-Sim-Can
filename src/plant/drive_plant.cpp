#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

void DrivePlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    if (dt_s <= 0.0) return;

    const bool enabled = cmd.system_enable;
    const double v = s.v_mps;

    // --- Resistive forces (always applied)
    const double F_drag = p_.drag_c * v * std::abs(v);   
    const double F_roll = p_.roll_c * static_cast<double>(sgn(v));   
    const double F_res = F_drag + F_roll;

    double Fx = 0.0;

    // --- Brake force calculation based on brake percentage
    const double brake_pct = clamp(cmd.brake_cmd_pct, 0.0, 100.0);
    const double brake_tq_mag = (brake_pct / 100.0) * p_.brake_torque_max_nm;
    const double brake_tq = brake_tq_mag * static_cast<double>(sgn(v));

    // Calculate brake force (in kN)
    const double brake_force_kN = brake_tq / p_.wheel_radius_m / 1000.0;

    LOG_DEBUG("[DrivePlant] v=%.2f m/s, motor_cmd=%.0f Nm, brake=%.1f%%", 
              v, cmd.drive_torque_cmd_nm, brake_pct);

    // Calculate motor torque command
    double motor_tq_cmd = clamp(cmd.drive_torque_cmd_nm, -p_.motor_torque_max_nm, +p_.motor_torque_max_nm);

    // Wheel torque from motor (apply drivetrain efficiency)
    double wheel_tq_from_motor = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff;

    // Power limiting: apply max power limits based on vehicle speed
    const double denom_v = std::max(std::abs(v), p_.v_stop_eps);
    const double wheel_tq_power_max = (p_.motor_power_max_w * p_.wheel_radius_m) / denom_v;
    wheel_tq_from_motor = clamp(wheel_tq_from_motor, -wheel_tq_power_max, +wheel_tq_power_max);

    // Net wheel torque (motor + brake)
    const double wheel_tq = wheel_tq_from_motor - brake_tq;
    Fx = wheel_tq / p_.wheel_radius_m;

    // --- Power demand calculation (Torque * Angular Velocity)
    double angular_velocity = v / p_.wheel_radius_m;  // radians per second
    double power_demand_kW = wheel_tq_from_motor * angular_velocity / 1000.0; // kW

    LOG_DEBUG("[DrivePlant] wheel_tq=%.2f Nm, omega=%.2f rad/s, P_demand=%.2f kW", 
              wheel_tq_from_motor, angular_velocity, power_demand_kW);

    // Initialize battery state fields
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = brake_force_kN;

    // --- Update battery and apply forces
    if (enabled) {
        // Update battery with power demand
        if (battery_plant_) {
            battery_plant_->step(power_demand_kW, brake_force_kN, dt_s);
            
            // Read back battery state into PlantState
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

    // --- Regenerative braking logic
    if (brake_pct > 0.0 && std::abs(v) > p_.v_stop_eps && motor_tq_cmd == 0.0) {
        // ACTIVE REGEN BRAKING (when brake pedal is pressed)
        // Higher efficiency (70%) for active braking
        const double regen_eff = 0.70;
        double regen_power_kW = brake_force_kN * std::abs(v) * regen_eff;
        
        if (battery_plant_) {
            // ====================================================================
            // CRITICAL FIX: Pass BOTH energy_J AND regen_power_kW parameters!
            // ====================================================================
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);  // ← Added 2nd parameter
            
            s.regen_power_kW = regen_power_kW;
            
            // Read back current from battery (now updated in store_energy)
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();  // Also update voltage
            
            LOG_DEBUG("[DrivePlant] Active Regen: P=%.2f kW, I=%.2f A, F=%.2f kN", 
                      regen_power_kW, s.batt_i, brake_force_kN);
        }
    } 
    else if (motor_tq_cmd == 0.0 && brake_pct == 0.0 && std::abs(v) > p_.v_stop_eps) {
        // COASTING REGEN (motor off, no brake, vehicle still moving)
        // Very low efficiency (5%) - most energy is lost to heat
        const double coasting_regen_eff = 0.05;  // REDUCED from 0.10
        
        // Resistance power dissipated
        const double resist_power_kW = F_res * std::abs(v) / 1000.0;  // kW
        
        // Only recover a small fraction
        double regen_power_kW = resist_power_kW * coasting_regen_eff;
        
        if (battery_plant_ && regen_power_kW > 0.001) {
            // ====================================================================
            // CRITICAL FIX: Pass BOTH energy_J AND regen_power_kW parameters!
            // ====================================================================
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);  // ← Added 2nd parameter
            
            s.regen_power_kW = regen_power_kW;
            
            // Read back current from battery (now updated in store_energy)
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();  // Also update voltage
            
            LOG_INFO("[DrivePlant] Coasting Regen: P=%.2f kW, I=%.2f A, F_res=%.2f kN", 
                      regen_power_kW, s.batt_i, F_res/1000.0);
        }
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