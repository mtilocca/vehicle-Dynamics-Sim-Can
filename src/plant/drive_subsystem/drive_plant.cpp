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
    
    // ========================================================================
    // TRACTION LIMITING (NEW - for dynamic model)
    // ========================================================================
    // When dynamic_model_enabled, the actual force applied is limited by
    // available tire traction (computed by TyreSubsystem via Dugoff model)
    
    double Fx_demanded = wheel_tq / p_.wheel_radius_m;
    
    if (s.dynamic_model_enabled) {
        // In dynamic mode, TyreSubsystem has already computed the friction-limited
        // tire forces. The VehicleBicycleAckermann will use those forces directly.
        // 
        // However, we still need to update wheel speeds to reflect the commanded
        // torque so the TyreSubsystem can compute slip ratios correctly.
        //
        // The actual velocity integration happens in VehicleBicycleAckermann
        // using the tire forces, not here.
        
        // Available traction force from tires (only rear wheels driven)
        double Fx_available = std::abs(s.Fx_rl) + std::abs(s.Fx_rr);
        
        // If demanded force exceeds available traction, we're traction-limited
        if (std::abs(Fx_demanded) > Fx_available && Fx_available > 0.0) {
            // Scale down to available traction
            Fx = std::copysign(Fx_available, Fx_demanded);
            
            LOG_DEBUG("[DrivePlant] TRACTION LIMITED: demanded=%.0f N, available=%.0f N",
                      Fx_demanded, Fx_available);
        } else {
            Fx = Fx_demanded;
        }
        
        // Store demanded force for TyreSubsystem to use in slip calculation
        // (The actual forces are computed by TyreSubsystem based on slip)
        
    } else {
        // KINEMATIC MODE: No traction limiting (V1 behavior)
        Fx = Fx_demanded;
    }

    // --- Power demand calculation (Torque * Angular Velocity)
    double angular_velocity = v / p_.wheel_radius_m;                          // radians per second
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
            // Case 1: Driving (power consumption)
            if (power_demand_kW > 0) {
                double energy_J = power_demand_kW * dt_s * 1000.0;
                battery_plant_->consume_energy(energy_J, power_demand_kW);

                s.motor_power_kW = power_demand_kW;

                // Read back current from battery
                s.batt_i = battery_plant_->get_current();
                s.batt_v = battery_plant_->get_voltage();

                LOG_DEBUG("[DrivePlant] Driving: P=%.2f kW, I=%.2f A, Fx=%.2f kN",
                          power_demand_kW, s.batt_i, Fx / 1000.0);
            }
            // Case 2: Regenerative braking (charging)
            else if (power_demand_kW < 0) {
                double regen_power_kW = -power_demand_kW * p_.regen_eff_active;

                double energy_J = regen_power_kW * dt_s * 1000.0;
                battery_plant_->store_energy(energy_J, regen_power_kW);

                s.motor_power_kW = power_demand_kW;
                s.regen_power_kW = regen_power_kW;

                // Read back current from battery (negative for charging)
                s.batt_i = battery_plant_->get_current();
                s.batt_v = battery_plant_->get_voltage();

                LOG_DEBUG("[DrivePlant] Regen: P=%.2f kW, I=%.2f A",
                          regen_power_kW, s.batt_i);
            }
            // Case 3: Coasting (resistive forces only, light regen)
            else if (std::abs(v) > p_.v_stop_eps && brake_pct < 1.0) {
                // Light regen from coasting deceleration
                double regen_power_kW = F_res * std::abs(v) / 1000.0 * p_.regen_eff_coast;

                double energy_J = regen_power_kW * dt_s * 1000.0;
                battery_plant_->store_energy(energy_J, regen_power_kW);

                s.regen_power_kW = regen_power_kW;

                // Read back current from battery
                s.batt_i = battery_plant_->get_current();
                s.batt_v = battery_plant_->get_voltage();

                LOG_DEBUG("[DrivePlant] Coasting Regen: P=%.2f kW, I=%.2f A, F_res=%.2f kN",
                          regen_power_kW, s.batt_i, F_res / 1000.0);
            }
        }

        // ====================================================================
        // VELOCITY INTEGRATION
        // ====================================================================
        // In KINEMATIC mode: Integrate velocity here using unlimited traction
        // In DYNAMIC mode: VehicleBicycleAckermann does velocity integration
        //                  using friction-limited tire forces
        
        if (!s.dynamic_model_enabled) {
            // --- KINEMATIC MODE: Net force and acceleration (V1 behavior) ---
            const double F_net = Fx - F_res;
            const double a = F_net / p_.mass_kg;

            double v_next = v + a * dt_s;
            v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

            // Snap to zero if speed crosses zero to avoid tiny oscillations
            if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
                if (std::abs(v_next) < 0.05)
                    v_next = 0.0;
            }

            s.a_long_mps2 = a;
            s.v_mps = v_next;
        }
        // else: Dynamic mode - velocity updated by VehicleBicycleAckermann
        
        s.motor_torque_nm = motor_tq_cmd;

        // Calculate wheel speeds from vehicle velocity
        // In dynamic mode, these will be used by TyreSubsystem for slip calculation
        double wheel_rps = s.v_mps / p_.wheel_radius_m;

        s.wheel_fl_rps = wheel_rps;
        s.wheel_fr_rps = wheel_rps;
        s.wheel_rl_rps = wheel_rps;
        s.wheel_rr_rps = wheel_rps;
    }
}

} // namespace plant