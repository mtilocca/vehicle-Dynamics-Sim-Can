// src/sim/sim_app.cpp - WITH SENSOR INTEGRATION
#include "sim/sim_app.hpp"
#include "sim/actuator_cmd.hpp"
#include "sim/plant_state_packer.hpp"
#include "sim/timing_controller.hpp"
#include "sensors/sensor_bank.hpp"  // NEW
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
#include <linux/can.h>

namespace sim {

SimApp::SimApp(SimAppConfig cfg) : cfg_(cfg), lua_() {
    if (cfg_.enable_debug_log_file) {
        utils::open_log_file(cfg_.debug_log_path);
    }
}

int SimApp::run_plant_only() {
    const double dt = cfg_.dt_s;
    
    // ========================================================================
    // Timing controller
    // ========================================================================
    TimingController timer(dt);
    double spin_threshold_us = (dt * 1e6) * 0.05;
    spin_threshold_us = std::max(20.0, std::min(100.0, spin_threshold_us));
    timer.set_spin_threshold_us(spin_threshold_us);
    
    LOG_INFO("[Timing] dt=%.4fs (%.0f us), spin_threshold=%.0f us", 
             dt, dt * 1e6, spin_threshold_us);
    
    // ========================================================================
    // Plant model initialization
    // ========================================================================
    plant::PlantModelParams pmp;
    
    if (cfg_.vehicle_params.has_value()) {
        pmp = cfg_.vehicle_params.value();
        LOG_INFO("[SimApp] Using vehicle params from YAML config");
    } else {
        // Use hardcoded defaults
        pmp.wheelbase_m = 2.8;
        pmp.track_width_m = 1.6;
        pmp.steer.delta_max_deg = 35.0;
        
        pmp.battery_params.capacity_kWh = 60.0;
        pmp.battery_params.efficiency_charge = 0.95;
        pmp.battery_params.efficiency_discharge = 0.95;
        pmp.battery_params.max_charge_power_kW = 50.0;
        pmp.battery_params.max_discharge_power_kW = 150.0;
        pmp.battery_params.min_soc = 0.05;
        pmp.battery_params.max_soc = 0.95;
        
        pmp.motor_params.max_power_kW = 300.0;
        pmp.motor_params.max_torque_nm = 4000.0;
        pmp.motor_params.efficiency = 0.92;
        
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
        
        LOG_INFO("[SimApp] Using hardcoded default vehicle params");
    }

    plant::PlantModel plant_model(pmp);
    plant::PlantState s{};
    s.batt_soc_pct = 50.0;

    // ========================================================================
    // Sensor bank initialization - NEW
    // ========================================================================
    sensors::SensorBankConfig sensor_cfg{};
    sensor_cfg.enable_battery_sensor = true;
    sensor_cfg.enable_wheel_sensor = true;
    
    // Configure battery sensor noise
    sensor_cfg.battery_params.voltage_noise_stddev = 0.5;      // 0.5V noise
    sensor_cfg.battery_params.current_noise_stddev = 0.2;      // 0.2A noise
    sensor_cfg.battery_params.soc_noise_stddev = 0.5;          // 0.5% noise
    sensor_cfg.battery_params.voltage_update_hz = 100.0;
    sensor_cfg.battery_params.current_update_hz = 100.0;
    sensor_cfg.battery_params.soc_update_hz = 1.0;             // Slow SOC update
    
    // Configure wheel sensor noise
    sensor_cfg.wheel_params.ticks_per_revolution = 48;         // 48-tick encoder
    sensor_cfg.wheel_params.noise_stddev_pct = 0.5;            // 0.5% noise
    sensor_cfg.wheel_params.update_hz = 100.0;
    
    sensors::SensorBank sensor_bank(sensor_cfg);
    LOG_INFO("[Sensors] Initialized %zu sensors", sensor_bank.sensor_count());

    // ---- CSV logging ----
    std::ofstream csv(cfg_.csv_log_path);
    if (!csv) {
        LOG_ERROR("Failed to open CSV: %s", cfg_.csv_log_path.c_str());
        return 1;
    }

    // CSV header with TRUTH and MEASURED columns
    csv << "t_s,"
        // Truth
        << "x_m,y_m,yaw_deg,v_mps,steer_deg,"
        << "delta_fl_deg,delta_fr_deg,motor_nm,brake_pct,"
        << "batt_soc_truth,batt_v_truth,batt_i_truth,"
        << "wheel_fl_rps_truth,wheel_fr_rps_truth,wheel_rl_rps_truth,wheel_rr_rps_truth,"
        << "motor_power_kW,regen_power_kW,brake_force_kN,"
        // Measured (sensors)
        << "batt_soc_meas,batt_v_meas,batt_i_meas,batt_temp_meas,"
        << "wheel_fl_rps_meas,wheel_fr_rps_meas,wheel_rl_rps_meas,wheel_rr_rps_meas,"
        // Timing
        << "loop_time_us,wall_time_s,time_drift_ms\n";
    
    csv << std::fixed << std::setprecision(6);

    // ---- Loop control ----
    ActuatorCmd cmd{};
    cmd.system_enable = true;

    const int max_iters = (cfg_.duration_s > 0.0) ?
        static_cast<int>(cfg_.duration_s / dt) : 0;

    const double log_period_s = (cfg_.log_hz > 0.0) ?
        1.0 / cfg_.log_hz : 0.1;
    double next_log = 0.0;

    // ---- CAN setup ----
    can::SocketCanIface can_iface;
    can::TxScheduler tx_scheduler;
    can::CanMap can_map;
    bool can_ready = false;

    if (cfg_.enable_can_tx) {
        if (can_map.load(cfg_.can_map_path)) {
            if (can_iface.open(cfg_.can_interface)) {
                tx_scheduler.init(can_map.tx_frames());
                can_ready = true;
                LOG_INFO("CAN TX enabled on %s", cfg_.can_interface.c_str());
            } else {
                LOG_WARN("Failed to open CAN interface: %s", cfg_.can_interface.c_str());
            }
        } else {
            LOG_WARN("Failed to load CAN map: %s", cfg_.can_map_path.c_str());
        }
    }

    // ---- Lua scenario setup ----
    if (cfg_.use_lua_scenario) {
        if (!lua_.init(cfg_.lua_script_path, cfg_.scenario_json_path)) {
            LOG_WARN("Failed to init Lua runtime");
            LOG_INFO("Falling back to open-loop default commands");
        } else {
            lua_ready_ = true;
            LOG_INFO("Lua scenario loaded: %s", cfg_.lua_script_path.c_str());
        }
    }

    // ---- Main loop ----
    LOG_INFO("Starting simulation loop (duration=%.1fs, dt=%.4fs)", cfg_.duration_s, dt);
    
    timer.reset();

    for (int iter = 0; (max_iters == 0) || (iter < max_iters); ++iter) {
        timer.mark_loop_start();
        const double t = cfg_.real_time_mode ? timer.get_sim_time() : (iter * dt);

        // ---- Generate actuator command ----
        if (lua_ready_) {
            if (!lua_.get_actuator_cmd(t, s, cmd)) {
                LOG_WARN("[t=%.2f] Lua get_actuator_cmd failed, using defaults", t);
                lua_ready_ = false;
            }
        }
        
        if (!lua_ready_) {
            cmd.drive_torque_cmd_nm = cfg_.motor_torque_nm;
            cmd.brake_cmd_pct = cfg_.brake_pct;
            cmd.steer_cmd_deg = cfg_.steer_amp_deg * std::sin(2.0 * M_PI * cfg_.steer_freq_hz * t);
        }

        // ---- Step plant (TRUTH) ----
        plant_model.step(s, cmd, dt);
        s.t_s = t;

        // ---- Step sensors (MEASURED) - NEW ----
        sensor_bank.step(t, s, dt);
        auto sensor_out = sensor_bank.get_output();

        // ---- CAN TX (send MEASURED data, not truth) ----
        if (can_ready) {
            auto now = std::chrono::steady_clock::now();
            auto due_indices = tx_scheduler.due(now);
            
            for (size_t idx : due_indices) {
                const auto& frame_def = can_map.tx_frames()[idx];
                
                // CRITICAL: Pack SENSOR data, not truth!
                // TODO: Update PlantStatePacker to accept SensorOut
                // For now, still using truth - will fix next
                auto signals = sim::PlantStatePacker::pack(s, frame_def);
                signals["loop_time_us"] = timer.get_last_loop_time_us();
                
                struct can_frame frame;
                can::CanCodec::encode_from_map(frame_def, signals, frame);
                can_iface.write_frame(frame);
            }
        }

        timer.update_loop_stats();

        // ---- Log to CSV (TRUTH + MEASURED) ----
        if (t >= next_log) {
            double steer_deg = s.steer_virtual_rad * 180.0 / M_PI;
            double delta_fl_deg = s.delta_fl_rad * 180.0 / M_PI;
            double delta_fr_deg = s.delta_fr_rad * 180.0 / M_PI;
            double yaw_deg = s.yaw_rad * 180.0 / M_PI;
            
            csv << s.t_s << ","
                // Truth
                << s.x_m << "," << s.y_m << "," << yaw_deg << ","
                << s.v_mps << "," << steer_deg << ","
                << delta_fl_deg << "," << delta_fr_deg << ","
                << cmd.drive_torque_cmd_nm << "," << cmd.brake_cmd_pct << ","
                << s.batt_soc_pct << "," << s.batt_v << "," << s.batt_i << ","
                << s.wheel_fl_rps << "," << s.wheel_fr_rps << "," 
                << s.wheel_rl_rps << "," << s.wheel_rr_rps << ","
                << s.motor_power_kW << "," << s.regen_power_kW << "," << s.brake_force_kN << ","
                // Measured (sensors)
                << sensor_out.batt_soc_meas << "," << sensor_out.batt_v_meas << "," 
                << sensor_out.batt_i_meas << "," << sensor_out.batt_temp_meas << ","
                << sensor_out.wheel_fl_rps_meas << "," << sensor_out.wheel_fr_rps_meas << ","
                << sensor_out.wheel_rl_rps_meas << "," << sensor_out.wheel_rr_rps_meas << ","
                // Timing
                << timer.get_last_loop_time_us() << ","
                << timer.get_wall_time() << ","
                << (timer.get_time_drift() * 1000.0) << "\n";

            next_log += log_period_s;
        }

        // ---- Real-time pacing ----
        if (cfg_.real_time_mode) {
            bool on_time = timer.wait_for_next_step();
            
            if (!on_time && (iter % 1000 == 0)) {
                auto stats = timer.get_stats();
                LOG_WARN("[t=%.2f] Deadline miss! Total misses: %zu, Max lateness: %.1f us",
                         t, stats.deadline_misses, stats.max_lateness_us);
            }
        }
    }

    // ---- Final timing statistics ----
    auto stats = timer.get_stats();
    
    LOG_INFO("========================================");
    LOG_INFO("Timing Statistics");
    LOG_INFO("========================================");
    LOG_INFO("Total steps: %zu", stats.total_steps);
    LOG_INFO("Deadline misses: %zu (%.2f%%)", 
             stats.deadline_misses,
             100.0 * stats.deadline_misses / stats.total_steps);
    LOG_INFO("Max loop time: %.1f us (%.1f%% of dt)",
             stats.max_loop_time_us,
             100.0 * stats.max_loop_time_us / (dt * 1e6));
    LOG_INFO("Max lateness: %.1f us", stats.max_lateness_us);
    if (stats.deadline_misses > 0) {
        LOG_INFO("Avg lateness: %.1f us", stats.avg_lateness_us);
    }
    LOG_INFO("Final time drift: %.3f ms", timer.get_time_drift() * 1000.0);
    LOG_INFO("========================================");

    LOG_INFO("Simulation complete. CSV written to: %s", cfg_.csv_log_path.c_str());
    csv.close();
    return 0;
}

} // namespace sim