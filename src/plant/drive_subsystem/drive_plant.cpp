// src/plant/drive_subsystem/drive_plant.cpp
//
// DrivePlant Implementation
//
// Key responsibilities:
// 1. Compute motor torque from command (with power limiting)
// 2. Populate tau_drive_rl_nm, tau_drive_rr_nm (RWD, 50/50 open diff)
// 3. Populate tau_brake_*_nm (40/60 front/rear bias)
// 4. Handle battery energy tracking
// 5. KINEMATIC mode only: integrate velocity, set wheel speeds
//
// Reference: Vehicle_Dynamics_with_Dugoff_Tire_Model.pdf (Section 5, Eq. 53)

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

    // ========================================================================
    // STEP 1: Motor Torque Calculation (PDF Eq. 53)
    // ========================================================================
    // τ_wheel = τ_motor × N × η
    // Where: N = gear ratio, η = drivetrain efficiency
    
    // Clamp motor command to limits
    double motor_tq_cmd = clamp(cmd.drive_torque_cmd_nm, 
                                -p_.motor_torque_max_nm, 
                                +p_.motor_torque_max_nm);

    // Wheel torque from motor (total for rear axle)
    double wheel_tq_total = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff;

    // ========================================================================
    // STEP 2: Power Limiting
    // ========================================================================
    // P = τ × ω, so τ_max = P_max / ω
    // At low speed, use v_stop_eps to avoid division by zero
    
    const double omega = std::max(std::abs(v), p_.v_stop_eps) / p_.wheel_radius_m;
    const double wheel_tq_power_max = p_.motor_power_max_w / omega;
    
    wheel_tq_total = clamp(wheel_tq_total, -wheel_tq_power_max, +wheel_tq_power_max);

    // ========================================================================
    // STEP 3: Brake Torque Distribution (40% front / 60% rear)
    // ========================================================================
    // Mining trucks have more weight on rear, need more rear braking
    
    const double brake_pct = clamp(cmd.brake_cmd_pct, 0.0, 100.0);
    const double brake_tq_total = (brake_pct / 100.0) * p_.brake_torque_max_nm;
    
    // Distribute to axles (total per axle, then split per wheel)
    const double brake_tq_front_axle = brake_tq_total * p_.brake_bias_front;
    const double brake_tq_rear_axle = brake_tq_total * (1.0 - p_.brake_bias_front);
    
    // Per-wheel brake torque (50/50 left-right)
    // Brake torque is always positive (opposes motion, WheelDynamics handles sign)
    s.tau_brake_fl_nm = brake_tq_front_axle * 0.5;
    s.tau_brake_fr_nm = brake_tq_front_axle * 0.5;
    s.tau_brake_rl_nm = brake_tq_rear_axle * 0.5;
    s.tau_brake_rr_nm = brake_tq_rear_axle * 0.5;
    
    // For backward compatibility / logging
    const double brake_force_kN = brake_tq_total / p_.wheel_radius_m / 1000.0;
    s.brake_force_kN = brake_force_kN;

    // ========================================================================
    // STEP 4: Drive Torque Distribution (RWD, 50/50 open diff)
    // ========================================================================
    // Front wheels: non-driven (tau_drive = 0)
    // Rear wheels: equal split (open differential)
    //
    // Future: Could add limited-slip differential or torque vectoring here
    
    s.tau_drive_fl_nm = 0.0;  // Front-left: non-driven
    s.tau_drive_fr_nm = 0.0;  // Front-right: non-driven
    s.tau_drive_rl_nm = wheel_tq_total * 0.5;  // Rear-left: 50%
    s.tau_drive_rr_nm = wheel_tq_total * 0.5;  // Rear-right: 50%

    LOG_DEBUG("[DrivePlant] v=%.2f m/s, τ_motor=%.0f Nm, τ_wheel_total=%.0f Nm, brake=%.1f%%",
              v, motor_tq_cmd, wheel_tq_total, brake_pct);
    LOG_DEBUG("[DrivePlant] τ_drive_rl=%.0f Nm, τ_drive_rr=%.0f Nm, τ_brake_fl=%.0f Nm",
              s.tau_drive_rl_nm, s.tau_drive_rr_nm, s.tau_brake_fl_nm);

    // ========================================================================
    // STEP 5: Power Demand and Battery Energy Flow
    // ========================================================================
    
    // Power = Torque × Angular velocity (at motor shaft)
    const double motor_omega = omega * p_.gear_ratio;
    double power_demand_kW = motor_tq_cmd * motor_omega / 1000.0;

    // Initialize battery state fields
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.motor_torque_nm = motor_tq_cmd;

    if (enabled && battery_plant_) {
        // Case 1: Driving (power consumption)
        if (power_demand_kW > 0.01) {
            double energy_J = power_demand_kW * dt_s * 1000.0;
            battery_plant_->consume_energy(energy_J);

            s.motor_power_kW = power_demand_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();

            LOG_DEBUG("[DrivePlant] Driving: P=%.2f kW, I=%.2f A", 
                      power_demand_kW, s.batt_i);
        }
        // Case 2: Regenerative braking (charging)
        else if (power_demand_kW < -0.01) {
            double regen_power_kW = -power_demand_kW * p_.regen_eff_active;
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);

            s.motor_power_kW = power_demand_kW;
            s.regen_power_kW = regen_power_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();

            LOG_DEBUG("[DrivePlant] Regen: P=%.2f kW, I=%.2f A", 
                      regen_power_kW, s.batt_i);
        }
        // Case 3: Coasting (light regen from drag)
        else if (std::abs(v) > p_.v_stop_eps && brake_pct < 1.0) {
            const double F_drag = p_.drag_c * v * std::abs(v);
            const double F_roll = p_.roll_c;
            const double F_res = F_drag + F_roll;
            
            double regen_power_kW = F_res * std::abs(v) / 1000.0 * p_.regen_eff_coast;
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);

            s.regen_power_kW = regen_power_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();
        }
    }

    // ========================================================================
    // STEP 6: Mode-Dependent Velocity/Wheel Speed Handling
    // ========================================================================
    
    // For backward compatibility / logging
    const double brake_force_kN = brake_tq_total / p_.wheel_radius_m / 1000.0;
    s.brake_force_kN = brake_force_kN;

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

    // Initialize battery state fields
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.motor_torque_nm = motor_tq_cmd;

    if (enabled && battery_plant_) {
        // Case 1: Driving (power consumption)
        if (power_demand_kW > 0.01) {
            double energy_J = power_demand_kW * dt_s * 1000.0;
            battery_plant_->consume_energy(energy_J);

            s.motor_power_kW = power_demand_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();

            LOG_DEBUG("[DrivePlant] Driving: P=%.2f kW, I=%.2f A", 
                      power_demand_kW, s.batt_i);
        }
        // Case 2: Regenerative braking (charging)
        else if (power_demand_kW < -0.01) {
            double regen_power_kW = -power_demand_kW * p_.regen_eff_active;
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);

            s.motor_power_kW = power_demand_kW;
            s.regen_power_kW = regen_power_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();

            LOG_DEBUG("[DrivePlant] Regen: P=%.2f kW, I=%.2f A", 
                      regen_power_kW, s.batt_i);
        }
        // Case 3: Coasting (light regen from drag)
        else if (std::abs(v) > p_.v_stop_eps && brake_pct < 1.0) {
            const double F_drag = p_.drag_c * v * std::abs(v);
            const double F_roll = p_.roll_c;
            const double F_res = F_drag + F_roll;
            
            double regen_power_kW = F_res * std::abs(v) / 1000.0 * p_.regen_eff_coast;
            double energy_J = regen_power_kW * dt_s * 1000.0;
            battery_plant_->store_energy(energy_J, regen_power_kW);

            s.regen_power_kW = regen_power_kW;
            s.batt_i = battery_plant_->get_current();
            s.batt_v = battery_plant_->get_voltage();
        }
    }

    // ========================================================================
    // STEP 6: Mode-Dependent Velocity/Wheel Speed Handling
    // ========================================================================
    
    if (s.dynamic_model_enabled) {
        // ====================================================================
        // DYNAMIC MODE: WheelSubsystem handles wheel speeds
        // ====================================================================
        // - We've already populated tau_drive_* and tau_brake_*
        // - WheelSubsystem will integrate wheel dynamics: Iw·ω̇ = τ - Fx·R
        // - WheelSubsystem will set omega_*_radps and wheel_*_rps
        // - VehicleBicycleAckermann will integrate velocity from ΣFx
        //
        // We do NOT set wheel_*_rps here in dynamic mode!
        // We do NOT integrate velocity here in dynamic mode!
        
        LOG_DEBUG("[DrivePlant] DYNAMIC mode: torques set, WheelSubsystem handles wheels");
        
    } else {
        // ====================================================================
        // KINEMATIC MODE: Classic V1 behavior (backward compatibility)
        // ====================================================================
        // - Integrate velocity directly (no slip, unlimited traction)
        // - Derive wheel speeds from vehicle velocity: ω = V/R
        
        // Resistive forces
        const double F_drag = p_.drag_c * v * std::abs(v);
        const double F_roll = p_.roll_c * static_cast<double>(sgn(v));
        const double F_res = F_drag + F_roll;
        
        // Drive force from torque (no traction limiting in kinematic mode)
        const double Fx = wheel_tq_total / p_.wheel_radius_m;
        
        // Brake force (opposes motion)
        const double F_brake = brake_tq_total / p_.wheel_radius_m * static_cast<double>(sgn(v));
        
        // Net force and acceleration
        const double F_net = Fx - F_brake - F_res;
        const double a = F_net / p_.mass_kg;

        // Velocity integration
        double v_next = v + a * dt_s;
        v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

        // Zero-crossing detection (prevent oscillation at standstill)
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

        LOG_DEBUG("[DrivePlant] KINEMATIC mode: v=%.2f m/s, a=%.2f m/s², ω=%.2f rad/s",
                  s.v_mps, s.a_long_mps2, wheel_omega);
    }

    // Wheel speeds
    const double wheel_rps = s.v_mps / p_.wheel_radius_m;
    s.wheel_fl_rps = wheel_rps;
    s.wheel_fr_rps = wheel_rps;
    s.wheel_rl_rps = wheel_rps;
    s.wheel_rr_rps = wheel_rps;
}

} // namespace plant
