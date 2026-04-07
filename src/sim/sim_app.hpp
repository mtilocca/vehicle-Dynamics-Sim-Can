// src/sim/sim_app.hpp
#pragma once

#include <string>
#include <optional>

#include "plant/plant_main/plant_state.hpp"
#include "plant/plant_model.hpp"

namespace sim {

struct SimAppConfig {
    // Core sim timing
    double dt_s = 0.01;
    double duration_s = 20.0;
    double log_hz = 10.0;

    // Real-time mode
    bool real_time_mode = true;

    // Output files
    std::string csv_log_path = "sim_out.csv";
    std::string debug_log_path = "sim_debug.log";

    // Logging control
    bool enable_debug_log_file = true;

    // CAN configuration
    bool enable_can_tx = true;
    std::string can_interface = "vcan0";
    std::string can_map_path = "config/can_map.dbc";

    // ========== CAN RX for closed-loop control ==========

    /**
     * Enable CAN RX for closed-loop control
     *
     * When true:
     * - Simulator listens for ACTUATOR_CMD_1 frames on CAN
     * - Actuator commands come from external controller
     *
     * When false (default):
     * - cmd stays at reset() — zero torque, zero brake, safe mode
     */
    bool enable_can_rx = false;

    /**
     * CAN frame name to listen for actuator commands
     * Must exist in can_map.dbc as RX frame
     * Default: "ACTUATOR_CMD_1" (J1939 CAN ID 0x18EFF021)
     */
    std::string actuator_cmd_frame_name = "ACTUATOR_CMD_1";

    /**
     * CAN RX timeout (seconds)
     *
     * If no CAN messages received for this duration, simulator enters safe mode:
     * - Zero torque
     * - Zero brake
     * - Zero steering
     * - System disabled
     *
     * Typical values: 0.1 - 1.0 seconds
     * Default: 0.5 seconds (10 missed frames at 20 Hz)
     */
    double can_rx_timeout_s = 0.5;

    // Vehicle parameters (always set from VehicleConfig::get_default())
    std::optional<plant::PlantModelParams> vehicle_params;

    // ========== Surface Friction Configuration ==========

    /**
     * Surface friction coefficient (mu)
     *
     * Common values (Pilbara mining conditions):
     *   - Dry pavement:         0.85
     *   - Gravel compact:       0.72 (default — heavy-duty haul roads)
     *   - Gravel loose:         0.55
     *   - Iron ore dust (dry):  0.45
     *   - Iron ore dust (wet):  0.30
     *   - Mud:                  0.25
     */
    double surface_friction = 0.72;

    // ========== InfluxDB Configuration ==========

    bool enable_influx = false;
    std::string influx_url = "http://localhost:8086";
    std::string influx_token = "";
    std::string influx_org = "Autonomy";
    std::string influx_bucket = "vehicle-sim";
    double influx_interval_s = 0.25;
};

class SimApp {
public:
    explicit SimApp(SimAppConfig cfg);

    int run_plant();

private:
    SimAppConfig cfg_;
};

} // namespace sim
