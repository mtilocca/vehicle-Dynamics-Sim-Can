#include "plant_model.hpp"
#include "vehicle_bicycle_ackermann.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"

namespace plant {

PlantModel::PlantModel(PlantModelParams p)
: p_(p),
  steer_(p_.steer),
  battery_(p_.battery_params, p_.motor_params),
  drive_(p_.drive, &battery_)
{
    // Initialize geometry parameters
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    
    // Set default battery parameters if not provided
    if (p_.battery_params.capacity_kWh == 0.0) {
        p_.battery_params.capacity_kWh = 60.0;  // 60 kWh battery
        p_.battery_params.efficiency_charge = 0.95;
        p_.battery_params.efficiency_discharge = 0.95;
        p_.battery_params.max_charge_power_kW = 50.0;
        p_.battery_params.max_discharge_power_kW = 150.0;
        p_.battery_params.min_soc = 0.05;  // 5% minimum
        p_.battery_params.max_soc = 0.95;  // 95% maximum
    }
    
    // Set default motor parameters if not provided
    if (p_.motor_params.max_power_kW == 0.0) {
        p_.motor_params.max_power_kW = 90.0;  // 90 kW motor
        p_.motor_params.max_torque_nm = 4000.0;
        p_.motor_params.efficiency = 0.92;
    }
    
    // Re-initialize battery with defaults
    battery_.set_params(p_.battery_params, p_.motor_params);
    
    LOG_INFO("[PlantModel] Initialized: Battery %.1f kWh, Motor %.1f kW", 
             p_.battery_params.capacity_kWh, p_.motor_params.max_power_kW);
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    steer_.params() = p_.steer;
    drive_.params() = p_.drive;
    battery_.set_params(p_.battery_params, p_.motor_params);
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;

    // Update steering
    steer_.step(s, cmd, dt_s);
    
    // Update drive (which updates battery)
    drive_.step(s, cmd, dt_s);

    // Update kinematic bicycle model
    BicycleAckermannParams ap{};
    ap.L_m = p_.wheelbase_m;
    ap.W_m = p_.track_width_m;
    ap.delta_max_rad = (p_.steer.delta_max_deg * 3.141592653589793 / 180.0);

    BicycleState2D pose{};
    pose.x_m = s.x_m;
    pose.y_m = s.y_m;
    pose.yaw_rad = s.yaw_rad;

    auto res = VehicleBicycleAckermann::step(pose, s.v_mps, s.steer_virtual_rad, ap, dt_s, battery_);

    s.x_m = res.next.x_m;
    s.y_m = res.next.y_m;
    s.yaw_rad = res.next.yaw_rad;
}

} // namespace plant