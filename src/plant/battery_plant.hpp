#pragma once

#include <cstdint>

namespace plant {

struct BatteryPlantParams {
    double capacity_kWh = 60.0;
    double efficiency_charge = 0.95;
    double efficiency_discharge = 0.95;
    double max_charge_power_kW = 50.0;
    double max_discharge_power_kW = 150.0;
    double min_soc = 0.05;
    double max_soc = 0.95;
};

struct MotorParams {
    double max_power_kW = 90.0;
    double max_torque_nm = 4000.0;
    double efficiency = 0.92;
};

class BatteryPlant {
public:
    BatteryPlant(const BatteryPlantParams& params = {}, const MotorParams& motor_params = {});

    void set_params(const BatteryPlantParams& params, const MotorParams& motor_params);
    
    void step(double power_demand_kW, double brake_force_kN, double dt_s);
    
    void consume_energy(double energy_consumed_J);
    
    // UPDATED: Now accepts regen_power_kW to fix current display
    void store_energy(double energy_stored_J, double regen_power_kW = 0.0);
    
    double regen_braking(double speed_mps, double brake_force_kN);
    
    double motor_power(double torque_nm, double speed_radps);

    double get_soc() const;
    double get_voltage() const;
    double get_current() const;
    double get_power() const;

private:
    void update_state(double power_demand_kW, double brake_force_kN, double dt_s);

    BatteryPlantParams params_;
    MotorParams motor_params_;
    
    double soc_;
    double voltage_;
    double current_;
    double power_;
    double regen_power_kW_;  // Track regen power for current calculation
};

} // namespace plant