// src/sim/sim_app.cpp - Simplified: CAN closed-loop + optional InfluxDB
#include "can/actuator_cmd_decoder.hpp"
#include "sim/sim_app.hpp"
#include "sim/actuator_cmd.hpp"
#include "sim/plant_state_packer.hpp"
#include "sim/timing_controller.hpp"
#include "can/socketcan_iface.hpp"
#include "can/tx_scheduler.hpp"
#include "can/can_map.hpp"
#include "can/can_codec.hpp"
#include "plant/plant_model.hpp"
#include "utils/logging.hpp"
#include "utils/influx.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>
#include <iomanip>
#include <linux/can.h>

namespace sim {

SimApp::SimApp(SimAppConfig cfg) : cfg_(cfg) {
    if (cfg_.enable_debug_log_file) {
        utils::open_log_file(cfg_.debug_log_path);
    }
}

int SimApp::run_plant() {
    const double dt = cfg_.dt_s;

    // ========================================================================
    // Timing controller
    // ========================================================================
    TimingController timer(dt);
    double spin_threshold_us = (dt * 1e6) * 0.05;
    spin_threshold_us = std::max(20.0, std::min(100.0, spin_threshold_us));
    timer.set_spin_threshold_us(spin_threshold_us);

    LOG_INFO("[SimApp] dt=%.4fs (%.0f us), spin_threshold=%.0f us",
             dt, dt * 1e6, spin_threshold_us);

    // ========================================================================
    // Plant model initialization
    // ========================================================================
    plant::PlantModelParams pmp;

    if (cfg_.vehicle_params.has_value()) {
        pmp = cfg_.vehicle_params.value();
        LOG_INFO("[SimApp] Using vehicle params from YAML config");
    } else {
        // Hardcoded defaults (XCMG XDE320-like)
        pmp.wheelbase_m   = 6.30;
        pmp.track_width_m = 7.20;

        pmp.steer.delta_max_deg = 35.0;

        pmp.drive.mass_kg            = 218000.0;
        pmp.drive.wheel_radius_m     = 1.93;
        pmp.drive.drag_c             = 2.5;
        pmp.drive.roll_c             = 1500.0;
        pmp.drive.motor_torque_max_nm = 9500.0;
        pmp.drive.motor_power_max_w   = 2800000.0;
        pmp.drive.brake_torque_max_nm = 80000.0;
        pmp.drive.gear_ratio          = 25.0;
        pmp.drive.drivetrain_eff      = 0.92;
        pmp.drive.v_stop_eps          = 0.5;
        pmp.drive.v_max_mps           = 17.78;
        pmp.drive.Cy_front_Npm        = 2500000.0;
        pmp.drive.Cy_rear_Npm         = 2000000.0;

        pmp.geometry.cg_height_m     = 3.20;
        pmp.geometry.cg_to_front_m   = 2.52;
        pmp.geometry.cg_to_rear_m    = 3.78;
        pmp.geometry.yaw_inertia_kgm2 = 8500000.0;

        LOG_INFO("[SimApp] Using hardcoded default vehicle params");
    }

    // Apply surface friction override
    pmp.drive.mu_surface = cfg_.surface_friction;
    LOG_INFO("[SimApp] Surface friction: mu=%.2f", cfg_.surface_friction);

    plant::PlantModel plant_model(pmp);
    plant::PlantState s{};

    // ========================================================================
    // InfluxDB setup
    // ========================================================================
    std::unique_ptr<utils::InfluxClient> influx_client;

    if (cfg_.enable_influx && !cfg_.real_time_mode) {
        LOG_WARN("[InfluxDB] InfluxDB requires --real-time mode. Disabling.");
        cfg_.enable_influx = false;
    }

    if (cfg_.enable_influx) {
        try {
            utils::InfluxClient::Config ic;
            ic.enabled          = true;
            ic.url              = cfg_.influx_url;
            ic.token            = cfg_.influx_token;
            ic.org              = cfg_.influx_org;
            ic.bucket           = cfg_.influx_bucket;
            ic.write_interval_s = cfg_.influx_interval_s;
            influx_client = std::make_unique<utils::InfluxClient>(ic);
            LOG_INFO("[InfluxDB] Logging enabled to %s/%s", cfg_.influx_org.c_str(), cfg_.influx_bucket.c_str());
        } catch (const std::exception& e) {
            LOG_ERROR("[InfluxDB] Init failed: %s — continuing without InfluxDB", e.what());
            cfg_.enable_influx = false;
        }
    }

    // ========================================================================
    // CSV logging setup
    // ========================================================================
    std::ofstream csv(cfg_.csv_log_path);
    if (!csv) {
        LOG_ERROR("Failed to open CSV: %s", cfg_.csv_log_path.c_str());
        return 1;
    }

    csv << "t_s,"
        "x_m,y_m,yaw_deg,v_mps,vy_mps,yaw_rate_radps,"
        "a_long_mps2,a_lat_mps2,"
        "steer_deg,delta_fl_deg,delta_fr_deg,"
        "motor_nm_cmd,brake_pct_cmd,motor_torque_nm,brake_force_kN,"
        "wheel_fl_rps,wheel_fr_rps,wheel_rl_rps,wheel_rr_rps,"
        "loop_time_us,wall_time_s\n";
    csv << std::fixed << std::setprecision(6);

    // ========================================================================
    // CAN setup
    // ========================================================================
    can::SocketCanIface can_iface;
    can::TxScheduler    tx_scheduler;
    can::CanMap         can_map;
    bool can_ready = false;

    if (cfg_.enable_can_tx || cfg_.enable_can_rx) {
        if (can_map.load(cfg_.can_map_path) && can_iface.open(cfg_.can_interface)) {
            if (cfg_.enable_can_tx) {
                tx_scheduler.init(can_map.tx_frames());
            }
            can_ready = true;
            LOG_INFO("[CAN] Interface opened: %s (%zu TX frames)",
                     cfg_.can_interface.c_str(), can_map.tx_frames().size());
        } else {
            LOG_WARN("[CAN] Failed to open interface or load map — CAN disabled");
        }
    }

    // ========================================================================
    // CAN RX decoder (closed-loop mode)
    // ========================================================================
    std::unique_ptr<can::ActuatorCmdDecoder> can_rx_decoder;
    double last_can_rx_time = -999.0;
    bool   can_rx_active    = false;

    if (cfg_.enable_can_rx) {
        if (!can_ready) {
            LOG_ERROR("[CAN RX] Requested but CAN not ready — aborting");
            return -1;
        }
        try {
            can_rx_decoder = std::make_unique<can::ActuatorCmdDecoder>(
                can_map, cfg_.actuator_cmd_frame_name);
            can_rx_active = true;
            LOG_INFO("[CAN RX] Enabled on frame %s (0x%03X), timeout=%.2fs",
                     cfg_.actuator_cmd_frame_name.c_str(),
                     can_rx_decoder->get_frame_id(),
                     cfg_.can_rx_timeout_s);
        } catch (const std::exception& e) {
            LOG_ERROR("[CAN RX] Init failed: %s", e.what());
            return -1;
        }
    }

    // ========================================================================
    // Main loop
    // ========================================================================
    ActuatorCmd cmd{};
    cmd.system_enable = true;   // Enabled by default; controller can override via CAN

    const int max_iters = (cfg_.duration_s > 0.0)
        ? static_cast<int>(cfg_.duration_s / dt) : 0;

    const double log_period_s = (cfg_.log_hz > 0.0) ? 1.0 / cfg_.log_hz : 0.1;
    double next_log = 0.0;

    LOG_INFO("[SimApp] Starting loop (duration=%.1fs, dt=%.4fs, log=%.1fHz)",
             cfg_.duration_s, dt, cfg_.log_hz);
    LOG_INFO("[SimApp] Mode: %s", can_rx_active ? "CLOSED-LOOP (CAN RX)" : "OPEN-LOOP (zero cmd)");

    timer.reset();

    for (int iter = 0; (max_iters == 0) || (iter < max_iters); ++iter) {
        timer.mark_loop_start();
        const double t = cfg_.real_time_mode ? timer.get_sim_time() : (iter * dt);

        // ====================================================================
        // Actuator command
        // ====================================================================
        if (can_rx_active && can_rx_decoder) {
            struct can_frame rx_frame;
            while (can_iface.read_nonblocking(rx_frame)) {
                if (can_rx_decoder->decode(rx_frame, cmd, t)) {
                    last_can_rx_time = t;
                }
            }
            // Safety: reset to zero on timeout
            if ((t - last_can_rx_time) > cfg_.can_rx_timeout_s) {
                if (iter % 1000 == 0) {
                    LOG_WARN("[t=%.2f] CAN RX timeout (%.2fs) — safe mode", t, t - last_can_rx_time);
                }
                cmd.reset();
            }
        }
        // else: cmd stays at zero (system_enable=true, zero torque/steer/brake)

        // ====================================================================
        // Step plant
        // ====================================================================
        plant_model.step(s, cmd, dt);
        s.t_s = t;

        // ====================================================================
        // InfluxDB
        // ====================================================================
        if (influx_client && influx_client->is_enabled()) {
            // Write simplified vehicle truth data
            influx_client->write_vehicle_truth(s, t);
        }

        // ====================================================================
        // CAN TX — plant-state frames only (0x220, 0x221, 0x300–0x3F0)
        // ====================================================================
        if (can_ready && cfg_.enable_can_tx) {
            auto now       = std::chrono::steady_clock::now();
            auto due_indices = tx_scheduler.due(now);

            for (size_t idx : due_indices) {
                const auto& frame_def = can_map.tx_frames()[idx];
                struct can_frame frame;
                frame.can_id  = frame_def.frame_id;
                frame.can_dlc = 8;

                auto signals = sim::PlantStatePacker::pack(s, frame_def);
                signals["loop_time_us"] = timer.get_last_loop_time_us();
                can::CanCodec::encode_from_map(frame_def, signals, frame);
                can_iface.write_frame(frame);
            }
        }

        timer.update_loop_stats();

        // ====================================================================
        // CSV logging
        // ====================================================================
        if (t >= next_log) {
            const double yaw_deg      = s.yaw_rad * 180.0 / M_PI;
            const double steer_deg    = s.steer_virtual_rad * 180.0 / M_PI;
            const double delta_fl_deg = s.delta_fl_rad * 180.0 / M_PI;
            const double delta_fr_deg = s.delta_fr_rad * 180.0 / M_PI;

            csv << s.t_s << ","
                << s.x_m << "," << s.y_m << "," << yaw_deg << ","
                << s.v_mps << "," << s.vy_mps << "," << s.yaw_rate_radps << ","
                << s.a_long_mps2 << "," << s.a_lat_mps2 << ","
                << steer_deg << "," << delta_fl_deg << "," << delta_fr_deg << ","
                << cmd.drive_torque_cmd_nm << "," << cmd.brake_cmd_pct << ","
                << s.motor_torque_nm << "," << s.brake_force_kN << ","
                << s.wheel_fl_rps << "," << s.wheel_fr_rps << ","
                << s.wheel_rl_rps << "," << s.wheel_rr_rps << ","
                << timer.get_last_loop_time_us() << ","
                << timer.get_wall_time()
                << "\n";

            next_log += log_period_s;
        }

        // ====================================================================
        // Real-time pacing
        // ====================================================================
        if (cfg_.real_time_mode) {
            bool on_time = timer.wait_for_next_step();
            if (!on_time && (iter % 1000 == 0)) {
                auto stats = timer.get_stats();
                LOG_WARN("[t=%.2f] Deadline miss! total=%zu, max_late=%.1f us",
                         t, stats.deadline_misses, stats.max_lateness_us);
            }
        }
    }

    // ========================================================================
    // Cleanup and statistics
    // ========================================================================
    if (influx_client) {
        influx_client->flush();
        LOG_INFO("[InfluxDB] Data flushed");
    }

    auto stats = timer.get_stats();
    LOG_INFO("========================================");
    LOG_INFO("Simulation complete");
    LOG_INFO("Steps: %zu, Deadline misses: %zu (%.2f%%)",
             stats.total_steps,
             stats.deadline_misses,
             100.0 * stats.deadline_misses / std::max(stats.total_steps, (size_t)1));
    LOG_INFO("Max loop time: %.1f us, Max lateness: %.1f us",
             stats.max_loop_time_us, stats.max_lateness_us);
    LOG_INFO("Final time drift: %.3f ms", timer.get_time_drift() * 1000.0);
    LOG_INFO("CSV written to: %s", cfg_.csv_log_path.c_str());
    LOG_INFO("========================================");

    csv.close();
    return 0;
}

} // namespace sim
