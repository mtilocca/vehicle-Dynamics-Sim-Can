// src/sim/sim_main.cpp
// UPDATED: Kinematic mode removed - Dynamic tire model (Dugoff) only
#include "sim/sim_app.hpp"
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <string>
#include <cstring>
#include <getopt.h>

// Parse log level string (case-insensitive) → LogLevel enum
// Returns true on success, false on invalid input
bool parse_log_level(const char* str, utils::LogLevel& out) {
    if (strcasecmp(str, "trace") == 0) { out = utils::LogLevel::Trace; return true; }
    if (strcasecmp(str, "debug") == 0) { out = utils::LogLevel::Debug; return true; }
    if (strcasecmp(str, "info")  == 0) { out = utils::LogLevel::Info;  return true; }
    if (strcasecmp(str, "warn")  == 0) { out = utils::LogLevel::Warn;  return true; }
    if (strcasecmp(str, "error") == 0) { out = utils::LogLevel::Error; return true; }
    if (strcasecmp(str, "off")   == 0) { out = utils::LogLevel::Off;   return true; }
    return false;
}

void print_usage(const char* prog_name) {
    printf("Usage: %s [options] [scenario.json]\n", prog_name);
    printf("\nSimulation Modes:\n");
    printf("  Default:          Open-loop with Lua/JSON scenario\n");
    printf("  --can-rx          Closed-loop, read actuator commands from CAN\n");
    printf("\nOptions:\n");
    printf("  --can-rx              Enable CAN RX for closed-loop control\n");
    printf("  --can-tx              Enable CAN TX (default: on)\n");
    printf("  --no-can-tx           Disable CAN TX\n");
    printf("  --can-iface NAME      CAN interface name (default: vcan0)\n");
    printf("  --can-timeout SEC     CAN RX timeout in seconds (default: 0.5)\n");
    printf("  --real-time           Run in real-time mode (default: from JSON)\n");
    printf("  --fast                Run as fast as possible (no real-time pacing)\n");
    printf("  --dt SEC              Timestep in seconds (default: from JSON)\n");
    printf("  --duration SEC        Simulation duration in seconds (default: from JSON)\n");
    printf("  --vehicle PATH        Vehicle config YAML (default: from JSON or built-in)\n");
    printf("  --can-map PATH        CAN signal map DBC (default: config/can_map.dbc)\n");
    printf("  --log-level LEVEL     Log verbosity: trace, debug, info, warn, error, off\n");
    printf("                        (default: info)\n");
    printf("  --help, -h            Show this help\n");
    
    printf("\nSurface Friction:\n");
    printf("  --surface-mu MU       Surface friction coefficient (default: 0.72)\n");
    printf("                        Common values:\n");
    printf("                          0.85 - Dry pavement\n");
    printf("                          0.72 - Compact gravel (default)\n");
    printf("                          0.55 - Loose gravel\n");
    printf("                          0.45 - Iron ore dust (dry)\n");
    printf("                          0.30 - Iron ore dust (wet)\n");
    printf("                          0.25 - Mud/slippery\n");
    
    printf("\nInfluxDB Options:\n");
    printf("  --influx              Enable InfluxDB time-series logging\n");
    printf("  --influx-url URL      InfluxDB server URL (default: http://localhost:8086)\n");
    printf("  --influx-token TOKEN  Authentication token (optional for local)\n");
    printf("  --influx-org ORG      Organization name (default: Autonomy)\n");
    printf("  --influx-bucket NAME  Bucket name (default: vehicle-sim)\n");
    printf("  --influx-interval MS  Write interval in ms (default: 250)\n");
    printf("\n  Note: InfluxDB logging requires --real-time mode\n");
    
    printf("\nTire Model:\n");
    printf("  Dynamic tire model (Dugoff) is always enabled.\n");
    printf("  Includes:\n");
    printf("    - Slip-based tire forces (Fx, Fy)\n");
    printf("    - Load transfer during acceleration/braking\n");
    printf("    - Friction saturation and combined slip\n");
    printf("    - Independent wheel dynamics\n");
    
    printf("\nExamples:\n");
    printf("  # Basic open-loop simulation:\n");
    printf("  %s config/scenarios/slalom.json\n\n", prog_name);
    
    printf("  # Wet surface conditions:\n");
    printf("  %s --surface-mu 0.30 config/scenarios/brake_test.json\n\n", prog_name);
    
    printf("  # Closed-loop control:\n");
    printf("  %s --can-rx --real-time --duration 60\n\n", prog_name);
    
    printf("  # Real-time with InfluxDB:\n");
    printf("  %s --real-time --influx config/scenarios/slalom.json\n\n", prog_name);
    
    printf("  # Fast-forward simulation:\n");
    printf("  %s --fast --duration 120 config/scenarios/accel.json\n\n", prog_name);
}

int main(int argc, char** argv) {
    // ========================================================================
    // Default configuration
    // ========================================================================
    sim::SimAppConfig cfg{};
    
    // Defaults (may be overridden by JSON or command-line)
    cfg.dt_s = 0.01;
    cfg.duration_s = 20.0;
    cfg.log_hz = 10.0;
    cfg.real_time_mode = false;
    
    cfg.csv_log_path = "sim_out.csv";
    cfg.debug_log_path = "sim_debug.log";
    cfg.enable_debug_log_file = true;

    cfg.enable_can_tx = true;
    cfg.enable_can_rx = false;
    cfg.can_interface = "vcan0";
    cfg.can_map_path = "config/can_map.dbc";
    cfg.actuator_cmd_frame_name = "ACTUATOR_CMD_1";
    cfg.can_rx_timeout_s = 0.5;
    
    // Surface friction (dynamic model always enabled)
    cfg.surface_friction = 0.72;  // Compact gravel default
    
    // InfluxDB defaults
    cfg.enable_influx = false;
    cfg.influx_url = "http://localhost:8086";
    cfg.influx_token = "";
    cfg.influx_org = "Autonomy";
    cfg.influx_bucket = "vehicle-sim";
    cfg.influx_interval_s = 0.25;  // 250ms = 4Hz
    
    utils::LogLevel log_level = utils::LogLevel::Info;

    // ========================================================================
    // Command-line parsing
    // ========================================================================
    static struct option long_options[] = {
        {"can-rx",          no_argument,       0, 'r'},
        {"can-tx",          no_argument,       0, 't'},
        {"no-can-tx",       no_argument,       0, 'T'},
        {"can-iface",       required_argument, 0, 'i'},
        {"can-map",         required_argument, 0, 'M'},
        {"can-timeout",     required_argument, 0, 'x'},
        {"real-time",       no_argument,       0, 'R'},
        {"fast",            no_argument,       0, 'F'},
        {"dt",              required_argument, 0, 'd'},
        {"duration",        required_argument, 0, 'D'},
        {"vehicle",         required_argument, 0, 'v'},
        {"surface-mu",      required_argument, 0, 'm'},
        {"log-level",       required_argument, 0, 'L'},
        // InfluxDB options
        {"influx",          no_argument,       0, 'I'},
        {"influx-url",      required_argument, 0, 'U'},
        {"influx-token",    required_argument, 0, 'K'},
        {"influx-org",      required_argument, 0, 'O'},
        {"influx-bucket",   required_argument, 0, 'B'},
        {"influx-interval", required_argument, 0, 'N'},
        {"help",            no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    int option_index = 0;
    
    while ((opt = getopt_long(argc, argv, "h", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'r':
                cfg.enable_can_rx = true;
                break;
            case 't':
                cfg.enable_can_tx = true;
                break;
            case 'T':
                cfg.enable_can_tx = false;
                break;
            case 'i':
                cfg.can_interface = optarg;
                break;
            case 'M':
                cfg.can_map_path = optarg;
                break;
            case 'x':
                cfg.can_rx_timeout_s = std::atof(optarg);
                if (cfg.can_rx_timeout_s <= 0) {
                    fprintf(stderr, "Error: Invalid CAN timeout: %s\n", optarg);
                    return 1;
                }
                break;
            case 'R':
                cfg.real_time_mode = true;
                break;
            case 'F':
                cfg.real_time_mode = false;
                break;
            case 'd':
                cfg.dt_s = std::atof(optarg);
                if (cfg.dt_s <= 0 || cfg.dt_s > 1.0) {
                    fprintf(stderr, "Error: Invalid timestep: %s (must be 0 < dt <= 1.0)\n", optarg);
                    return 1;
                }
                break;
            case 'D':
                cfg.duration_s = std::atof(optarg);
                if (cfg.duration_s <= 0) {
                    fprintf(stderr, "Error: Invalid duration: %s\n", optarg);
                    return 1;
                }
                break;
            case 'L':
                if (!parse_log_level(optarg, log_level)) {
                    fprintf(stderr, "Error: Invalid log level: '%s'\n", optarg);
                    fprintf(stderr, "  Valid levels: trace, debug, info, warn, error, off\n");
                    return 1;
                }
                break;
            case 'm':
                cfg.surface_friction = std::atof(optarg);
                if (cfg.surface_friction <= 0 || cfg.surface_friction > 1.5) {
                    fprintf(stderr, "Error: Invalid surface friction: %s (must be 0 < mu <= 1.5)\n", optarg);
                    return 1;
                }
                break;
            // InfluxDB options
            case 'I':
                cfg.enable_influx = true;
                break;
            case 'U':
                cfg.influx_url = optarg;
                break;
            case 'K':
                cfg.influx_token = optarg;
                break;
            case 'O':
                cfg.influx_org = optarg;
                break;
            case 'B':
                cfg.influx_bucket = optarg;
                break;
            case 'N':
                cfg.influx_interval_s = std::atof(optarg) / 1000.0;
                if (cfg.influx_interval_s <= 0) {
                    fprintf(stderr, "Error: Invalid InfluxDB interval: %s\n", optarg);
                    return 1;
                }
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }
    
    // ========================================================================
    // Validation
    // ========================================================================
    if (cfg.enable_can_rx && !cfg.enable_can_tx) {
        fprintf(stderr, "Warning: CAN RX enabled without CAN TX - controller won't receive sensor data!\n");
    }
    
    if (cfg.enable_influx && !cfg.real_time_mode) {
        fprintf(stderr, "Warning: InfluxDB logging requires --real-time mode. Disabling InfluxDB.\n");
        cfg.enable_influx = false;
    }

    utils::set_level(log_level);

    // ========================================================================
    // Vehicle configuration (always XCMG XDE320 hardcoded defaults)
    // ========================================================================
    config::VehicleConfig vehicle = config::VehicleConfig::get_default();
    vehicle.print_summary();
    cfg.vehicle_params = vehicle.params;

    // ========================================================================
    // Print configuration summary
    // ========================================================================
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║     PLANT-SENSOR-CAN SIMULATION - DYNAMIC TIRE MODEL      ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║ Mode:       %-48s║\n", cfg.enable_can_rx ? "CLOSED-LOOP (CAN RX)" : "SAFE (cmd zeroed)");
    printf("║ CAN TX:     %-48s║\n", cfg.enable_can_tx ? "enabled" : "disabled");
    printf("║ CAN RX:     %-48s║\n", cfg.enable_can_rx ? "enabled" : "disabled");
    printf("║ Interface:  %-48s║\n", cfg.can_interface.c_str());
    if (cfg.enable_can_rx) {
        char timeout_str[50];
        snprintf(timeout_str, sizeof(timeout_str), "%.2f seconds", cfg.can_rx_timeout_s);
        printf("║ RX Timeout: %-48s║\n", timeout_str);
        printf("║ RX Frame:   %-48s║\n", cfg.actuator_cmd_frame_name.c_str());
    }
    char timestep_str[50], duration_str[50];
    snprintf(timestep_str, sizeof(timestep_str), "%.4f seconds", cfg.dt_s);
    snprintf(duration_str, sizeof(duration_str), "%.1f seconds", cfg.duration_s);
    printf("║ Timestep:   %-48s║\n", timestep_str);
    printf("║ Duration:   %-48s║\n", duration_str);
    printf("║ Real-time:  %-48s║\n", cfg.real_time_mode ? "yes (1:1 wall clock)" : "no (fast-forward)");
    printf("║ Vehicle:    %-48s║\n", vehicle.name.c_str());
    
    // Tire model info
    char tire_model_str[60];
    snprintf(tire_model_str, sizeof(tire_model_str), "DYNAMIC (Dugoff, mu=%.2f)", cfg.surface_friction);
    printf("║ Tire Model: %-48s║\n", tire_model_str);
    
    if (cfg.enable_influx) {
        char influx_str[100];
        snprintf(influx_str, sizeof(influx_str), "%s/%s (%.0fms)", 
                 cfg.influx_org.c_str(), 
                 cfg.influx_bucket.c_str(),
                 cfg.influx_interval_s * 1000.0);
        printf("║ InfluxDB:   %-48s║\n", influx_str);
    } else {
        printf("║ InfluxDB:   %-48s║\n", "disabled");
    }
    
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    if (cfg.enable_can_rx) {
        printf("⚠️  CLOSED-LOOP MODE: Waiting for CAN commands on %s\n", cfg.can_interface.c_str());
        printf("    Frame: %s (J1939 ID: 0x18EFF021)\n", cfg.actuator_cmd_frame_name.c_str());
        printf("    Start your controller now!\n\n");
    }
    
    // Surface condition display
    printf("🚗 Dynamic Tire Model: Dugoff force-based dynamics\n");
    printf("   Surface: mu = %.2f", cfg.surface_friction);
    if (cfg.surface_friction >= 0.80) printf(" (dry pavement)\n");
    else if (cfg.surface_friction >= 0.65) printf(" (compact gravel)\n");
    else if (cfg.surface_friction >= 0.50) printf(" (loose gravel)\n");
    else if (cfg.surface_friction >= 0.40) printf(" (iron ore dust - dry)\n");
    else if (cfg.surface_friction >= 0.28) printf(" (iron ore dust - wet)\n");
    else printf(" (mud/slippery)\n");
    printf("   Features:\n");
    printf("     ✓ Slip-based tire forces (Fx, Fy)\n");
    printf("     ✓ Load transfer (acceleration/braking)\n");
    printf("     ✓ Friction saturation (λ < 1)\n");
    printf("     ✓ Independent wheel dynamics\n");
    printf("     ✓ Combined longitudinal/lateral slip\n");
    printf("\n");
    
    if (cfg.enable_influx) {
        printf("📊 InfluxDB: Logging to %s → %s/%s\n", 
               cfg.influx_url.c_str(),
               cfg.influx_org.c_str(),
               cfg.influx_bucket.c_str());
        if (!cfg.influx_token.empty()) {
            printf("    Using authentication token: %s...\n", 
                   cfg.influx_token.substr(0, 8).c_str());
        } else {
            printf("    No authentication (local mode)\n");
        }
        printf("\n");
    }

    // ========================================================================
    // Run simulation
    // ========================================================================
    LOG_INFO("========================================");
    LOG_INFO("Plant-Sensor-CAN Simulation");
    LOG_INFO("Vehicle: %s", vehicle.name.c_str());
    LOG_INFO("Tire Model: DYNAMIC (Dugoff)");
    LOG_INFO("Surface friction: mu=%.2f", cfg.surface_friction);
    LOG_INFO("========================================");

    sim::SimApp app(cfg);
    return app.run_plant();
}