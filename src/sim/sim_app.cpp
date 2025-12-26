// src/sim/sim_app.cpp
#include "sim/sim_app.hpp"
#include "sim/actuator_cmd.hpp"
#include "sim/plant_state_packer.hpp"
#include "can/socketcan_iface.hpp"
#include "can/tx_scheduler.hpp"
#include "can/can_map.hpp"
#include "can/can_codec.hpp"
#include "plant/plant_model.hpp"
#include "utils/logging.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>
#include <iomanip>

namespace sim {

SimApp::SimApp(SimAppConfig cfg) : cfg_(cfg), lua_() {
    if (cfg_.enable_debug_log_file) {
        utils::open_log_file(cfg_.debug_log_path);
    }
}

int SimApp::run_plant_only() {
    const double dt = cfg_.dt_s;
    
    // ---- Plant model initialization ----
    plant::PlantModelParams pmp{};
    pmp.wheelbase_m = 2.8;
    pmp.track_width_m = 1.6;
    pmp.steer.delta_max_deg = 35.0;
    
    // Battery parameters
    pmp.battery_params.capacity_kWh = 60.0;
    pmp.battery_params.efficiency_charge = 0.95;
    pmp.battery_params.efficiency_discharge = 0.95;
    pmp.battery_params.max_charge_power_kW = 50.0;
    pmp.battery_params.max_discharge_power_kW = 150.0;
    pmp.battery_params.min_soc = 0.05;
    pmp.battery_params.max_soc = 0.95;
    
    // Motor parameters
    pmp.motor_params.max_power_kW = 300.0;
    pmp.motor_params.max_torque_nm = 4000.0;
    pmp.motor_params.efficiency = 0.92;
    
    // Drive parameters
    pmp.drive.mass_kg = 1800.0;
    pmp.drive.wheel_radius_m = 0.33;
    pmp.drive.drag_c = 0.35;
    pmp.drive.roll_c = 40.0;
    pmp.drive.motor_torque_max_nm = 4000.0;
    pmp.drive.brake_torque_max_nm = 4000.0;
    pmp.drive.gear_ratio = 9.0;
    pmp.drive.drivetrain_eff = 0.92;
    pmp.drive.motor_power_max_w = 300000.0;
    pmp.drive.v_stop_eps = 0.3;
    pmp.drive.v_max_mps = 60.0;

    plant::PlantModel plant_model(pmp);
    plant::PlantState s{};
    s.batt_soc_pct = 50.0;

    // ---- CSV logging ----
    std::ofstream csv(cfg_.csv_log_path);
    if (!csv) {
        LOG_ERROR("Failed to open CSV: %s", cfg_.csv_log_path.c_str());
        return 1;
    }

    csv << "t_s,x_m,y_m,yaw_deg,v_mps,steer_deg,"
        << "delta_fl_deg,delta_fr_deg,motor_nm,brake_pct,"
        << "batt_soc_pct,batt_v,batt_i,motor_power_kW,regen_power_kW,brake_force_kN\n";
    csv << std::fixed << std::setprecision(6);

    // ---- Loop control ----
    ActuatorCmd cmd{};
    cmd.system_enable = true;

    const int max_iters = (cfg_.duration_s > 0.0) ?
        static_cast<int>(cfg_.duration_s / dt) : 0;

    const double log_period_s = (cfg_.log_hz > 0.0) ? (1.0 / cfg_.log_hz) : 0.1;
    double next_log_t = 0.0;

    // ---- Real-time pacing setup ----
    using Clock = std::chrono::steady_clock;
    using Nanoseconds = std::chrono::nanoseconds;
    
    auto sim_start_time = Clock::now();
    auto next_step_time = sim_start_time;

    // ---- CAN TX setup ----
    can::SocketCanIface can_tx;
    can::TxScheduler tx_sched;
    can::CanMap can_map;
    bool can_enabled = false;
    uint64_t can_tx_count = 0;
    
    // CRITICAL: Store plant_frames persistently so indices match!
    std::vector<can::FrameDef> plant_frames;

    if (cfg_.enable_can_tx && !cfg_.can_interface.empty()) {
        LOG_INFO("Attempting to open CAN interface: %s", cfg_.can_interface.c_str());
        
        if (!can_map.load(cfg_.can_map_path)) {
            LOG_WARN("Failed to load CAN map: %s", cfg_.can_map_path.c_str());
        } else if (!can_tx.open(cfg_.can_interface)) {
            LOG_WARN("Failed to open CAN interface: %s (continuing without CAN)", 
                     cfg_.can_interface.c_str());
        } else {
            // Collect ALL plant_state frames
            for (const auto& frame : can_map.tx_frames()) {
                // Check if ANY signal has target "plant_state"
                bool is_plant_frame = false;
                for (const auto& sig : frame.signals) {
                    if (sig.target == "plant_state") {
                        is_plant_frame = true;
                        break;
                    }
                }
                
                if (is_plant_frame) {
                    plant_frames.push_back(frame);
                    LOG_INFO("Scheduling TX frame: 0x%03X (%s) @ %d ms", 
                             frame.frame_id, frame.frame_name.c_str(), frame.cycle_ms);
                }
            }
            
            if (!plant_frames.empty()) {
                tx_sched.init(plant_frames);
                tx_sched.force_all_due();
                can_enabled = true;
                LOG_INFO("CAN TX enabled: %zu plant_state frames scheduled", plant_frames.size());
            } else {
                LOG_WARN("No plant_state frames found in CAN map");
            }
        }
    }

    // ---- Scenario init
    lua_ready_ = false;
    if (cfg_.use_lua_scenario) {
        lua_ready_ = lua_.init(cfg_.lua_script_path, cfg_.scenario_json_path);
        if (!lua_ready_) {
            LOG_WARN("Lua scenario requested but failed to init. Falling back to defaults.");
        }
    }

    LOG_INFO("Starting simulation...");
    LOG_INFO("Duration: %.1f s, dt: %.3f s, real-time: %s, CAN TX: %s",
             cfg_.duration_s, dt,
             cfg_.real_time_mode ? "ON" : "OFF",
             can_enabled ? "ON" : "OFF");

    int iter = 0;

    while (max_iters == 0 || iter < max_iters) {
        const double t = s.t_s;

        // ========== Actuator commands ==========
        if (lua_ready_) {
            if (!lua_.get_actuator_cmd(t, s, cmd)) {
                cmd.drive_torque_cmd_nm = cfg_.motor_torque_nm;
                cmd.brake_cmd_pct = cfg_.brake_pct;
                cmd.steer_cmd_deg = cfg_.steer_amp_deg * std::sin(2.0 * M_PI * cfg_.steer_freq_hz * t);
            }
        } else {
            cmd.drive_torque_cmd_nm = cfg_.motor_torque_nm;
            cmd.brake_cmd_pct = cfg_.brake_pct;
            cmd.steer_cmd_deg =
                cfg_.steer_amp_deg * std::sin(2.0 * M_PI * cfg_.steer_freq_hz * t);
        }

        plant_model.step(s, cmd, dt);

        // ========== CAN TX ==========
        if (can_enabled) {
            auto now = Clock::now();
            auto due_frames = tx_sched.due(now);
            
            for (size_t idx : due_frames) {
                // CRITICAL FIX: Use plant_frames[idx] not can_map.tx_frames()[idx]!
                const auto& frame_def = plant_frames[idx];
                
                // Pack PlantState → signals
                auto signals = PlantStatePacker::pack(s, frame_def);
                
                if (!signals.empty()) {
                    // Encode → CAN frame
                    struct can_frame frame;
                    can::CanCodec::encode_from_map(frame_def, signals, frame);
                    
                    // Transmit
                    if (can_tx.write_frame(frame)) {
                        ++can_tx_count;
                        
                        // Debug log (only first 20)
                        if (can_tx_count <= 20) {
                            LOG_DEBUG("TX 0x%03X (%s) with %zu signals", 
                                     frame_def.frame_id, frame_def.frame_name.c_str(), signals.size());
                        }
                    }
                } else {
                    // Empty signal map - log warning
                    if (can_tx_count < 10) {
                        LOG_WARN("Empty signal map for frame 0x%03X (%s)", 
                                frame_def.frame_id, frame_def.frame_name.c_str());
                    }
                }
            }
        }
        // ============================

        if (s.t_s >= next_log_t) {
            const double yaw_deg = s.yaw_rad * 180.0 / M_PI;
            const double steer_deg = s.steer_virtual_rad * 180.0 / M_PI;
            const double fl_deg = s.delta_fl_rad * 180.0 / M_PI;
            const double fr_deg = s.delta_fr_rad * 180.0 / M_PI;

            // CSV log
            if (csv.is_open()) {
                csv << s.t_s << ","
                    << s.x_m << ","
                    << s.y_m << ","
                    << yaw_deg << ","
                    << s.v_mps << ","
                    << steer_deg << ","
                    << fl_deg << ","
                    << fr_deg << ","
                    << s.motor_torque_nm << ","
                    << cmd.brake_cmd_pct << ","
                    << s.batt_soc_pct << ","
                    << s.batt_v << ","
                    << s.batt_i << ","
                    << s.motor_power_kW << ","
                    << s.regen_power_kW << ","
                    << s.brake_force_kN << "\n";
            }

            next_log_t += log_period_s;
        }

        // Real-time pacing
        if (cfg_.real_time_mode) {
            next_step_time += Nanoseconds(static_cast<long long>(dt * 1e9));
            std::this_thread::sleep_until(next_step_time);
        }

        ++iter;
        
        // Progress logging every 1000 iterations
        if (iter % 1000 == 0) {
            LOG_INFO("t=%.2f s, v=%.2f m/s, SOC=%.1f%%, CAN TX: %llu",
                     s.t_s, s.v_mps, s.batt_soc_pct, (unsigned long long)can_tx_count);
        }
    }

    LOG_INFO("Simulation complete: t=%.2f s", s.t_s);
    LOG_INFO("Final state: x=%.1f m, y=%.1f m, yaw=%.1f deg, v=%.2f m/s, SOC=%.1f%%",
             s.x_m, s.y_m, s.yaw_rad * 180.0 / M_PI, s.v_mps, s.batt_soc_pct);
    
    if (can_enabled) {
        LOG_INFO("Total CAN frames transmitted: %llu", (unsigned long long)can_tx_count);
    }

    return 0;
}

} // namespace sim