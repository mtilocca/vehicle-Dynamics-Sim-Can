// src/plant/drive_subsystem/drive_plant.hpp
#pragma once

#include <algorithm>
#include "plant/plant_main/plant_state.hpp"
#include "plant/battery_subsystem/battery_plant.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct DriveParams {
    // --- Vehicle Mass ---
    double mass_kg = 1800.0;
    
    // --- Wheel/Tire ---
    double wheel_radius_m = 0.33;
    
    // --- Resistive Forces ---
    double drag_c = 0.35;   // N per (m/s)^2 - aerodynamic drag coefficient
    double roll_c = 40.0;   // N (Coulomb-ish rolling resistance magnitude)
    
    // --- Motor Limits ---
    double motor_torque_max_nm = 4000.0;  // motor shaft torque clamp
    double motor_power_max_w = 300000;    // 300 kW max power
    
    // --- Brake Limits ---
    double brake_torque_max_nm = 4000.0;  // wheel brake torque at tire (magnitude)
    
    // --- Drivetrain ---
    double gear_ratio = 9.0;
    double drivetrain_eff = 0.92;
    
    // --- Regenerative Braking Efficiency ---
    double regen_eff_active = 0.68;   // Efficiency during active regen braking
    double regen_eff_coast = 0.03;    // Efficiency during coasting (light regen)
    
    // --- Speed Limits ---
    double v_stop_eps = 0.3;          // Velocity threshold for standstill logic (m/s)
    double v_max_mps = 60.0;          // Maximum velocity (m/s)
};

class DrivePlant {
public:
    explicit DrivePlant(DriveParams p = {}, BatteryPlant* battery_plant = nullptr)
        : p_(p), battery_plant_(battery_plant) {}

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const DriveParams& params() const { return p_; }
    DriveParams& params() { return p_; }

    /**
     * set_battery_plant() - Update battery connection at runtime
     * CRITICAL FIX: Needed for subsystem architecture dependency injection
     */
    void set_battery_plant(BatteryPlant* battery) { battery_plant_ = battery; }

private:
    DriveParams p_;
    BatteryPlant* battery_plant_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }

    double power_demand_kW_ = 0.0;
    double regen_power_kW_ = 0.0;
};

} // namespace plant