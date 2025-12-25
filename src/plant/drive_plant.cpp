#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include <iostream>
#include <cmath>
#include <cstdio>

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
    const double brake_force_kN = brake_tq / p_.wheel_radius_m / 1000.0;  // Convert to kN

    std::cout << "Drive Torque Command (Nm): " << cmd.drive_torque_cmd_nm << std::endl;


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

    std::cout << "wheel_tq_from_motor: " << wheel_tq_from_motor << std::endl;


    // --- Power demand calculation (Torque * Angular Velocity)
    double angular_velocity = v / p_.wheel_radius_m;  // radians per second
    std::cout << "angular_velocity: " << angular_velocity << std::endl;

    double power_demand_kW = wheel_tq_from_motor * angular_velocity / 1000.0; // kW

    // Log the power demand for debugging
    std::cout << "Power Demand (kW): " << power_demand_kW << std::endl;

    if (enabled) {
        // Pass the power demand to the battery
        battery_plant_->step(power_demand_kW, brake_force_kN, dt_s);  // Update BatteryPlant with power demand
    } else {
        // No motor power, apply only brake force
        Fx = (-brake_tq) / p_.wheel_radius_m;
    }

    // --- Regenerative braking logic
    if (cmd.brake_cmd_pct > 0.0) {
        // Calculate regen power based on the braking percentage
        double regen_power_kW = (cmd.brake_cmd_pct / 100.0) * power_demand_kW;
        battery_plant_->store_energy(regen_power_kW * dt_s);  // Store energy in the battery during braking
        Fx = -regen_power_kW_;  // Apply regen braking force to decelerate the vehicle
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
}

} // namespace plant