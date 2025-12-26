// src/plant/battery_subsystem.cpp
#include "plant/battery_subsystem.hpp"
#include "utils/logging.hpp"

namespace plant {

BatterySubsystem::BatterySubsystem(
    const BatteryPlantParams& battery_params,
    const MotorParams& motor_params
)
    : battery_(battery_params, motor_params),
      params_(battery_params),
      motor_params_(motor_params)
{
}

void BatterySubsystem::initialize(PlantState& s) {
    LOG_INFO("[BatterySubsystem] Initializing: %.1f kWh, %.1f kW max power",
             params_.capacity_kWh, motor_params_.max_power_kW);

    // Set initial battery state
    s.batt_soc_pct = battery_.get_soc() * 100.0;
    s.batt_v = battery_.get_voltage();
    s.batt_i = battery_.get_current();
}

void BatterySubsystem::reset(PlantState& s) {
    LOG_INFO("[BatterySubsystem] Resetting to 50%% SOC");
    
    // Reset internal battery state
    battery_ = BatteryPlant(params_, motor_params_);
    
    // Update PlantState
    s.batt_soc_pct = 50.0;
    s.batt_v = 400.0;
    s.batt_i = 0.0;
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
}

void BatterySubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Battery step is driven by DriveSubsystem via request_power()
    // This step() just synchronizes PlantState with internal battery state
    
    s.batt_soc_pct = battery_.get_soc() * 100.0;
    s.batt_v = battery_.get_voltage();
    s.batt_i = battery_.get_current();
    
    LOG_DEBUG("[BatterySubsystem] SOC=%.1f%%, V=%.1f V, I=%.2f A, P_req=%.2f kW",
              s.batt_soc_pct, s.batt_v, s.batt_i, power_request_kW_);
}

double BatterySubsystem::request_power(double power_kW, double dt) {
    power_request_kW_ = power_kW;
    
    // Clamp to battery limits
    const double max_discharge = params_.max_discharge_power_kW;
    const double max_charge = params_.max_charge_power_kW;
    
    double actual_power_kW = power_kW;
    if (power_kW > max_discharge) {
        actual_power_kW = max_discharge;
        LOG_DEBUG("[BatterySubsystem] Power limited: %.1f kW -> %.1f kW (discharge limit)",
                  power_kW, actual_power_kW);
    } else if (power_kW < -max_charge) {
        actual_power_kW = -max_charge;
        LOG_DEBUG("[BatterySubsystem] Power limited: %.1f kW -> %.1f kW (charge limit)",
                  power_kW, actual_power_kW);
    }
    
    // Update battery (brake_force not used in this path, set to 0)
    battery_.step(actual_power_kW, 0.0, dt);
    
    return actual_power_kW;
}

void BatterySubsystem::store_energy(double energy_J, double regen_power_kW) {
    battery_.store_energy(energy_J, regen_power_kW);
}

double BatterySubsystem::get_available_power_kW() const {
    // Available power depends on SOC
    const double soc = battery_.get_soc();
    
    // Discharge power limited at low SOC
    if (soc < params_.min_soc + 0.05) {
        return motor_params_.max_power_kW * 0.5; // Derating
    }
    
    // Charge power limited at high SOC
    if (soc > params_.max_soc - 0.05) {
        return motor_params_.max_power_kW * 0.3; // Reduced regen
    }
    
    return motor_params_.max_power_kW;
}

void BatterySubsystem::set_params(
    const BatteryPlantParams& battery_params,
    const MotorParams& motor_params
) {
    params_ = battery_params;
    motor_params_ = motor_params;
    battery_.set_params(battery_params, motor_params);
}

} // namespace plant
