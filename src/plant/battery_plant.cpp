#include "battery_plant.hpp"
#include <algorithm>
#include <cmath>

namespace plant {

BatteryPlant::BatteryPlant(const BatteryPlantParams& params, const MotorParams& motor_params)
    : params_(params),
      motor_params_(motor_params),
      soc_(1.0),
      voltage_(400.0),
      current_(0.0),
      power_(0.0) {}

void BatteryPlant::set_params(const BatteryPlantParams& params, const MotorParams& motor_params) {
    params_ = params;
    motor_params_ = motor_params;
}

void BatteryPlant::step(double power_demand_kW, double brake_force_kN, double dt_s) {
    update_state(power_demand_kW, brake_force_kN, dt_s);
}

void BatteryPlant::update_state(double power_demand_kW, double brake_force_kN, double dt_s) {
    if (dt_s <= 0.0) return;

    double power_demand_W = power_demand_kW * 1000.0;

    // NOTE: your regen_braking() signature must match your header declaration
    // If your header says regen_braking(double speed_mps, double brake_force_kN),
    // then don't call regen_braking(soc_, ...). Use a consistent signature.
    double regen_power_W = regen_braking(0.0 /*speed placeholder*/, brake_force_kN);

    power_demand_W = std::clamp(
        power_demand_W,
        -motor_params_.max_power_kW * 1000.0,
         motor_params_.max_power_kW * 1000.0
    );

    const double eff = (power_demand_W > 0.0) ? params_.efficiency_discharge
                                              : params_.efficiency_charge;

    // Convert to energy over dt (Wh) and then to SOC fraction
    const double energy_Wh = (power_demand_W * dt_s) / 3600.0;          // W*s -> Wh
    const double cap_Wh    = params_.capacity_kWh * 1000.0;             // kWh -> Wh
    const double delta_soc = -(energy_Wh / cap_Wh) * eff;               // discharge reduces SOC

    soc_ = std::clamp(soc_ + delta_soc, params_.min_soc, params_.max_soc);

    power_ = power_demand_W - regen_power_W;
}

void BatteryPlant::consume_energy(double energy_consumed_J) {
    // If you use this: pass Joules (W*s). Convert to Wh.
    const double energy_Wh = energy_consumed_J / 3600.0;
    const double cap_Wh = params_.capacity_kWh * 1000.0;
    soc_ = std::clamp(soc_ - (energy_Wh / cap_Wh), params_.min_soc, params_.max_soc);
}

void BatteryPlant::store_energy(double energy_stored_J) {
    const double energy_Wh = energy_stored_J / 3600.0;
    const double cap_Wh = params_.capacity_kWh * 1000.0;
    soc_ = std::clamp(soc_ + (energy_Wh / cap_Wh), params_.min_soc, params_.max_soc);
}

double BatteryPlant::regen_braking(double speed_mps, double brake_force_kN) {
    // placeholder simple model
    const double brake_force_N = brake_force_kN * 1000.0;
    const double mech_power_W = brake_force_N * std::abs(speed_mps);
    return mech_power_W * motor_params_.efficiency;
}

double BatteryPlant::motor_power(double torque_nm, double speed_radps) {
    return torque_nm * speed_radps;
}

double BatteryPlant::get_soc() const { return soc_; }
double BatteryPlant::get_voltage() const { return voltage_; }
double BatteryPlant::get_current() const { return current_; }
double BatteryPlant::get_power() const { return power_; }

} // namespace plant
