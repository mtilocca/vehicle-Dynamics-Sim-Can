#pragma once

#include <string>

namespace plant {

struct BatteryPlantParams {
    double capacity_kWh;
    double efficiency_charge;
    double efficiency_discharge;
    double max_charge_power_kW;
    double max_discharge_power_kW;
    double min_soc;
    double max_soc;
};

struct MotorParams {
    double max_power_kW;
    double max_torque_nm;
    double efficiency;
};

class BatteryPlant {
public:
    BatteryPlant(const BatteryPlantParams& params, const MotorParams& motor_params);

    void step(double power_demand_kW, double brake_force_kN, double dt_s);
    void consume_energy(double energy_consumed_W);
    void store_energy(double energy_stored_W);

    void set_params(const BatteryPlantParams& params, const MotorParams& motor_params);
    
    double get_soc() const;
    double get_voltage() const;
    double get_current() const;
    double get_power() const;

private:
    BatteryPlantParams params_;
    MotorParams motor_params_;

    double soc_;
    double voltage_;
    double current_;
    double power_;

    void update_state(double power_demand_kW, double brake_force_kN, double dt_s);
    double regen_braking(double speed_mps, double brake_force_kN);
    double motor_power(double torque_nm, double speed_mps);
};

} // namespace plant
