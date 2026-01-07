// src/plant/battery_plant.cpp
#include "battery_plant.hpp"
#include "utils/logging.hpp"
#include <algorithm>
#include <cmath>

namespace plant {

BatteryPlant::BatteryPlant(const BatteryPlantParams& params, const MotorParams& motor_params)
    : params_(params),
      motor_params_(motor_params),
      soc_(0.5),  // Start at 50% SOC
      voltage_(400.0),  // Initial voltage (will be updated in step())
      current_(0.0),
      power_(0.0),
      regen_power_kW_(0.0) {}

void BatteryPlant::set_params(const BatteryPlantParams& params, const MotorParams& motor_params) {
    params_ = params;
    motor_params_ = motor_params;
}

void BatteryPlant::step(double power_demand_kW, double brake_force_kN, double dt_s) {
    LOG_DEBUG("[BatteryPlant::step] P_demand=%.2f kW, brake=%.2f kN, dt=%.4f s", 
              power_demand_kW, brake_force_kN, dt_s);
    
    // Reset regen power tracker at start of each step
    regen_power_kW_ = 0.0;
    
    update_state(power_demand_kW, brake_force_kN, dt_s);
}

void BatteryPlant::update_state(double power_demand_kW, double brake_force_kN, double dt_s) {
    if (dt_s <= 0.0) return;

    // ========================================================================
    // CRITICAL FIX: Update voltage based on SOC FIRST before using it
    // ========================================================================
    // Voltage model: V = V_nominal * (0.85 + 0.30 * SOC)
    // At SOC=0%: V = 400 * 0.85 = 340V
    // At SOC=50%: V = 400 * (0.85 + 0.15) = 400V
    // At SOC=100%: V = 400 * 1.15 = 460V
    const double nominal_voltage = 400.0;
    voltage_ = nominal_voltage * (0.85 + 0.30 * soc_);
    
    // Convert power from kW to W for calculations
    double power_demand_W = power_demand_kW * 1000.0;

    LOG_DEBUG("[BatteryPlant::update_state] P_demand_W=%.1f W, V=%.1f V (SOC=%.1f%%)", 
              power_demand_W, voltage_, soc_ * 100.0);

    // Clamp power demand to motor limits
    power_demand_W = std::clamp(
        power_demand_W,
        -motor_params_.max_power_kW * 1000.0,
         motor_params_.max_power_kW * 1000.0
    );

    // Select efficiency based on charge/discharge
    const double eff = (power_demand_W > 0.0) ? params_.efficiency_discharge
                                              : params_.efficiency_charge;

    // Calculate energy consumed/generated over timestep
    // Energy (Wh) = Power (W) * time (h)
    const double energy_Wh = (power_demand_W * dt_s) / 3600.0;  // W*s -> Wh
    const double cap_Wh = params_.capacity_kWh * 1000.0;         // kWh -> Wh
    
    // Update SOC (discharge reduces SOC, charge increases it)
    const double delta_soc = -(energy_Wh / cap_Wh) * eff;
    soc_ = std::clamp(soc_ + delta_soc, params_.min_soc, params_.max_soc);

    // Update power and current
    // If regen is active, subtract it from power demand
    power_ = power_demand_W - (regen_power_kW_ * 1000.0);
    
    // Current = Power / Voltage
    // Positive current = discharge, Negative current = charge
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

void BatteryPlant::store_energy(double energy_stored_J, double regen_power_kW) {
    // ========================================================================
    // CRITICAL FIX: Update voltage based on current SOC before calculating current
    // ========================================================================
    const double nominal_voltage = 400.0;
    voltage_ = nominal_voltage * (0.85 + 0.30 * soc_);
    
    // Convert Joules (W*s) to Wh
    const double energy_Wh = energy_stored_J / 3600.0;
    const double cap_Wh = params_.capacity_kWh * 1000.0;
    
    soc_ = std::clamp(soc_ + (energy_Wh / cap_Wh), params_.min_soc, params_.max_soc);
    
    // CRITICAL FIX: Store regen power for current calculation in next step
    regen_power_kW_ = regen_power_kW;
    
    // Update current immediately to reflect charging
    if (regen_power_kW > 0.001) {
        power_ = -(regen_power_kW * 1000.0);  // Negative = charging
        current_ = power_ / voltage_;          // Will be negative
    }
    
    LOG_DEBUG("[BatteryPlant::store_energy] Stored %.2f J (%.4f Wh), P_regen=%.2f kW, V=%.1f V, I=%.2f A, SOC=%.3f", 
              energy_stored_J, energy_Wh, regen_power_kW, voltage_, current_, soc_);
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

void BatteryPlant::reset() {
    soc_ = 0.5;  // Reset to 50% SOC
    voltage_ = 400.0;
    current_ = 0.0;
    power_ = 0.0;
    regen_power_kW_ = 0.0;
}

const BatteryPlantParams& BatteryPlant::params() const {
    return params_;
}

const MotorParams& BatteryPlant::motor_params() const {
    return motor_params_;
}

} // namespace plant