#include "plant_model.hpp"

#include "vehicle_bicycle_ackermann.hpp"
#include "sim/actuator_cmd.hpp"

namespace plant {

PlantModel::PlantModel(PlantModelParams p)
: p_(p),
  steer_(p_.steer),
  drive_(p_.drive) {
    // keep geometry consistent
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    steer_.set_params(p_.steer);
    drive_.set_params(p_.drive);
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    // advance sim time
    s.t_s += dt_s;

    // 1) steering (rate limits + ackermann wheel angles)
    steer_.step(s, cmd, dt_s);

    // 2) longitudinal drive/brake -> update v + derived wheel speeds
    drive_.step(s, cmd, dt_s);

    // 3) pose integration with kinematic bicycle (rear axle reference)
    BicycleAckermannParams ap{};
    ap.L_m = p_.wheelbase_m;
    ap.W_m = p_.track_width_m;
    ap.delta_max_rad = (p_.steer.delta_max_deg * 3.141592653589793 / 180.0);

    BicycleState2D pose{};
    pose.x_m = s.x_m;
    pose.y_m = s.y_m;
    pose.yaw_rad = s.yaw_rad;

    auto res = VehicleBicycleAckermann::step(pose, s.v_mps, s.steer_virtual_rad, ap, dt_s);

    s.x_m = res.next.x_m;
    s.y_m = res.next.y_m;
    s.yaw_rad = res.next.yaw_rad;

    // Note: wheel angles already updated by SteerPlant (we keep those as the source)
}

} // namespace plant
