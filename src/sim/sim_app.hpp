// src/sim/sim_app.hpp
#pragma once

#include <string>
#include <optional>

#include "plant/plant_main/plant_state.hpp"
#include "plant/plant_model.hpp"

namespace sim {

struct SimAppConfig {
    // Core sim timing
    double dt_s       = 0.01;
    double duration_s = 0.0;   // 0 = run indefinitely
    double log_hz     = 10.0;

    // Real-time mode (1:1 wall-clock pacing)
    bool real_time_mode = true;

    // Output files
    std::string csv_log_path   = "sim_out.csv";
    std::string debug_log_path = "sim_debug.log";
    bool enable_debug_log_file = true;

    // CAN configuration
    bool enable_can_tx = true;
    std::string can_interface = "vcan0";
    std::string can_map_path  = "config/can_map.csv";

    // CAN RX — closed-loop control from external controller
    bool enable_can_rx = false;
    std::string actuator_cmd_frame_name = "ACTUATOR_CMD_1";
    double can_rx_timeout_s = 0.5;

    // Vehicle configuration (optional — uses hardcoded defaults if not set)
    std::optional<plant::PlantModelParams> vehicle_params;

    // Surface friction coefficient (written to DriveParams.mu_surface)
    double surface_friction = 0.72;

    // InfluxDB time-series logging (optional, requires real-time mode)
    bool enable_influx       = false;
    std::string influx_url    = "http://localhost:8086";
    std::string influx_token  = "";
    std::string influx_org    = "Autonomy";
    std::string influx_bucket = "vehicle-sim";
    double influx_interval_s  = 0.25;
};

class SimApp {
public:
    explicit SimApp(SimAppConfig cfg);

    int run_plant();

private:
    SimAppConfig cfg_;
};

} // namespace sim
