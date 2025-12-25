#pragma once

#include <string>

namespace plant {

struct BatteryPlantParams {
    double capacity_kWh;  // Total capacity of the battery (kWh)
    double efficiency_charge;  // Charging efficiency (0 to 1)
    double efficiency_discharge;  // Discharging efficiency (0 to 1)
    double max_charge_power_kW;  // Maximum charge rate (kW)
    double max_discharge_power_kW;  // Maximum discharge rate (kW)
    double min_soc;  // Minimum state of charge (SOC) (0 to 1)
    double max_soc;  // Maximum state of charge (SOC) (0 to 1)
};

struct MotorParams {
    double max_power_kW;  // Maximum power output of the motor (kW)
    double max_torque_nm;  // Maximum torque output of the motor (Nm)
    double efficiency;  // Motor efficiency (0 to 1)
};

class BatteryPlant {
public:
    BatteryPlant(const BatteryPlantParams& params, const MotorParams& motor_params);

    void step(double power_demand_kW, double brake_force_kN, double dt_s);  // Update battery and motor state based on power demand and regen braking

    // New functions for energy consumption and storage
    void consume_energy(double energy_consumed_W);  // Energy consumed during acceleration (discharge)
    void store_energy(double energy_stored_W);      // Energy stored during regenerative braking (charge)

 // New function to set parameters
    void set_params(const BatteryPlantParams& params, const MotorParams& motor_params);
    
    double get_soc() const;  // Get current state of charge (SOC)
    double get_voltage() const;  // Get the current voltage
    double get_current() const;  // Get the current
    double get_power() const;  // Get the current power output

private:
    BatteryPlantParams params_;
    MotorParams motor_params_;

    double soc_;  // Current state of charge (SOC)
    double voltage_;  // Battery voltage (V)
    double current_;  // Battery current (A)
    double power_;  // Power (W)

    void update_state(double power_demand_kW, double brake_force_kN, double dt_s);  // Update SOC, motor power, and battery state
    double regen_braking(double speed_mps, double brake_force_kN);  // Calculate regen braking power
    double motor_power(double torque_nm, double speed_mps);  // Calculate motor power
};

} // namespace plant
