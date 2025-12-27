// src/plant/plant_model.cpp
#include "plant/plant_model.hpp"
#include "plant/vehicle_bicycle_ackermann.hpp"
#include "plant/battery_subsystem.hpp"
#include "plant/drive_subsystem.hpp"
#include "plant/steer_subsystem.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"

namespace plant {

PlantModel::PlantModel(PlantModelParams p)
    : p_(p),
      subsystem_mgr_()
{
    // Initialize geometry parameters
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    
    
    LOG_INFO("[PlantModel] Initializing with SubsystemManager architecture");
    LOG_INFO("[PlantModel] Battery: %.1f kWh, Motor: %.1f kW, Mass: %.0f kg",
             p_.battery_params.capacity_kWh, p_.motor_params.max_power_kW, p_.drive.mass_kg);

    // ========================================================================
    // Register Subsystems (auto-sorted by priority)
    // ========================================================================
    
    // 1. SteerSubsystem (Priority 50 - Actuators)
    auto steer = std::make_unique<SteerSubsystem>(p_.steer);
    subsystem_mgr_.register_subsystem(std::move(steer));
    
    // 2. BatterySubsystem (Priority 150 - Energy)
    auto battery = std::make_unique<BatterySubsystem>(
        p_.battery_params,
        p_.motor_params
    );
    auto* battery_ptr = battery.get();  // Keep pointer for Drive dependency injection
    subsystem_mgr_.register_subsystem(std::move(battery));
    
    // 3. DriveSubsystem (Priority 100 - Dynamics)
    auto drive = std::make_unique<DriveSubsystem>(p_.drive);
    drive->set_battery_subsystem(battery_ptr);  // Inject battery dependency
    subsystem_mgr_.register_subsystem(std::move(drive));
    
    // Initialize all subsystems with default state
    PlantState init_state{};
    init_state.batt_soc_pct = 50.0;
    subsystem_mgr_.initialize_all(init_state);
    
    LOG_INFO("[PlantModel] Registered %zu subsystems", subsystem_mgr_.subsystem_count());
}

void PlantModel::set_params(const PlantModelParams& p) {
    p_ = p;
    p_.steer.wheelbase_m = p_.wheelbase_m;
    p_.steer.track_width_m = p_.track_width_m;
    
    // Update subsystem parameters
    auto* steer = subsystem_mgr_.find_subsystem("Steer");
    if (steer) {
        static_cast<SteerSubsystem*>(steer)->set_params(p_.steer);
    }
    
    auto* drive = subsystem_mgr_.find_subsystem("Drive");
    if (drive) {
        static_cast<DriveSubsystem*>(drive)->set_params(p_.drive);
    }
    
    auto* battery = subsystem_mgr_.find_subsystem("Battery");
    if (battery) {
        static_cast<BatterySubsystem*>(battery)->set_params(
            p_.battery_params,
            p_.motor_params
        );
    }
}

void PlantModel::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    s.t_s += dt_s;

    // Execute all subsystems (pre_step → step → post_step)
    // Order: Steer (50) → Drive (100) → Battery (150)
    subsystem_mgr_.step_all(s, cmd, dt_s);

    // Update kinematic bicycle model for position/yaw
    BicycleAckermannParams ap{};
    ap.L_m = p_.wheelbase_m;
    ap.W_m = p_.track_width_m;
    ap.delta_max_rad = (p_.steer.delta_max_deg * 3.141592653589793 / 180.0);

    BicycleState2D pose{};
    pose.x_m = s.x_m;
    pose.y_m = s.y_m;
    pose.yaw_rad = s.yaw_rad;

    // Note: VehicleBicycleAckermann::step still takes BatteryPlant reference
    // This is OK - it's wrapped inside BatterySubsystem
    // Future: Could refactor to not need this, but works fine for now
    auto* battery_sub = subsystem_mgr_.find_subsystem("Battery");
    if (battery_sub) {
        // Access internal BatteryPlant through subsystem (requires friend access or getter)
        // For now, bicycle step doesn't actually modify battery, so we can pass a dummy
        BatteryPlant dummy_battery(p_.battery_params, p_.motor_params);
        auto res = VehicleBicycleAckermann::step(pose, s.v_mps, s.steer_virtual_rad, ap, dt_s, dummy_battery);
        
        s.x_m = res.next.x_m;
        s.y_m = res.next.y_m;
        s.yaw_rad = res.next.yaw_rad;
    }
}

} // namespace plant