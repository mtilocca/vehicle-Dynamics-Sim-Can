// src/plant/battery_plant.cpp
// FINAL FIX: Update current_ in store_energy() for regen braking
#include "battery_plant.hpp"
#include "utils/logging.hpp"
#include <algorithm>
#include <cmath>

namespace plant {

BatteryPlant::BatteryPlant(const BatteryPlantParams& params, const MotorParams& motor_params)
    : params_(params),
      motor_params_(motor_params),
      soc_(0.5),  // Start at 50% SOC
      voltage_(400.0),
      current_(0.0),
      power_(0.0) {}

void BatteryPlant::set_params(const BatteryPlantParams& params, const MotorParams& motor_params) {
    params_ = params;
    motor_params_ = motor_params;
}

void BatteryPlant::step(double power_demand_kW, double brake_force_kN, double dt_s) {
    LOG_DEBUG("[BatteryPlant::step] P_demand=%.2f kW, brake=%.2f kN, dt=%.4f s", 
              power_demand_kW, brake_force_kN, dt_s);
    update_state(power_demand_kW, brake_force_kN, dt_s);
}

void BatteryPlant::update_state(double power_demand_kW, double brake_force_kN, double dt_s) {
    if (dt_s <= 0.0) return;

    // Convert power from kW to W for calculations
    double power_demand_W = power_demand_kW * 1000.0;

    LOG_DEBUG("[BatteryPlant::update_state] P_demand_W=%.1f W", power_demand_W);

    // Clamp power demand to motor limits
    power_demand_W = std::clamp(
        power_demand_W,
        -motor_params_.max_power_kW * 1000.0,
         motor_params_.max_power_kW * 1000.0
    );

    // Select efficiency based on charge/discharge
    const double eff = (power_demand_W > 0.0) 
                                              ? params_.efficiency_discharge
                                              : params_.efficiency_charge;

    // Calculate energy consumed/generated over timestep
    // Energy (Wh) = Power (W) * time (h)
    const double energy_Wh = (power_demand_W * dt_s) / 3600.0;  // W*s -> Wh
    const double cap_Wh = params_.capacity_kWh * 1000.0;         // kWh -> Wh
    
    // Update SOC (discharge reduces SOC, charge increases it)
    const double delta_soc = -(energy_Wh / cap_Wh) * eff;
    soc_ = std::clamp(soc_ + delta_soc, params_.min_soc, params_.max_soc);

    // Update voltage (simplified model - voltage depends on SOC)
    // Typical Li-ion: ~300-420V range
    voltage_ = 300.0 + (420.0 - 300.0) * soc_;
    
    // Update power (net power after accounting for efficiency)
    power_ = power_demand_W;
    
    // Current = Power / Voltage
    // Positive current = discharge (battery providing power to motor)
    // Negative current = charge (motor providing power to battery via regen)
    current_ = power_ / voltage_;

    LOG_DEBUG("[BatteryPlant::update_state] SOC=%.3f (%.1f%%), V=%.1f V, I=%.2f A, P=%.1f W", 
              soc_, soc_ * 100.0, voltage_, current_, power_);
}

void BatteryPlant::consume_energy(double energy_consumed_J) {
    // Convert Joules (W*s) to Wh
    const double energy_Wh = energy_consumed_J / 3600.0;
    const double cap_Wh = params_.capacity_kWh * 1000.0;
    
    soc_ = std::clamp(soc_ - (energy_Wh / cap_Wh), params_.min_soc, params_.max_soc);
    
    LOG_DEBUG("[BatteryPlant::consume_energy] Consumed %.2f J (%.4f Wh), SOC=%.3f", 
              energy_consumed_J, energy_Wh, soc_);
}

void BatteryPlant::store_energy(double energy_stored_J) {
    // ========================================================================
    // BUG FIX: Update current when storing regen energy
    // ========================================================================
    // Convert Joules (W*s) to Wh
    const double energy_Wh = energy_stored_J / 3600.0;
    const double cap_Wh = params_.capacity_kWh * 1000.0;
    
    soc_ = std::clamp(soc_ + (energy_Wh / cap_Wh), params_.min_soc, params_.max_soc);
    
    // CRITICAL FIX: Calculate and update current for regen
    // Energy (J) was stored, so calculate equivalent power and current
    // Assuming this energy was stored over a small timestep (use 0.01s as typical)
    const double assumed_dt_s = 0.01;  // 10ms timestep
    const double regen_power_W = energy_stored_J / assumed_dt_s;  // P = E / t
    
    // Update voltage based on new SOC
    voltage_ = 300.0 + (420.0 - 300.0) * soc_;
    
    // Update power (negative = charging)
    power_ = -regen_power_W;  // Negative because we're charging
    
    // Update current (negative = charging)
    current_ = power_ / voltage_;  // Will be negative!
    
    LOG_DEBUG("[BatteryPlant::store_energy] Stored %.2f J (%.4f Wh), SOC=%.3f, I=%.2f A", 
              energy_stored_J, energy_Wh, soc_, current_);
}

double BatteryPlant::regen_braking(double speed_mps, double brake_force_kN) {
    // Simple regenerative braking model
    // Mechanical power = Force * Velocity
    const double brake_force_N = brake_force_kN * 1000.0;
    const double mech_power_W = brake_force_N * std::abs(speed_mps);
    
    // Apply motor efficiency
    const double regen_power_W = mech_power_W * motor_params_.efficiency;
    
    LOG_DEBUG("[BatteryPlant::regen_braking] speed=%.2f m/s, F=%.2f kN, P_regen=%.2f W", 
              speed_mps, brake_force_kN, regen_power_W);
    
    return regen_power_W;
}

double BatteryPlant::motor_power(double torque_nm, double speed_radps) {
    return torque_nm * speed_radps;
}

double BatteryPlant::get_soc() const { return soc_; }
double BatteryPlant::get_voltage() const { return voltage_; }
double BatteryPlant::get_current() const { return current_; }
double BatteryPlant::get_power() const { return power_; }

} // namespace plant