// src/sim/sim_main.cpp
// UPDATED: Added CAN RX support and InfluxDB logging with command-line flags
#include "sim/sim_app.hpp"
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <getopt.h>

// Parse timing section from JSON (handles both old and new formats)
bool load_timing_from_json(const std::string& scenario_path, sim::SimAppConfig& cfg) {
    std::ifstream file(scenario_path);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    bool in_timing_section = false;
    bool found_timing = false;
    
    while (std::getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        if (line.find("\"timing\"") != std::string::npos) {
            in_timing_section = true;
            found_timing = true;
            continue;
        }
        
        if (in_timing_section && line.find("}") != std::string::npos) {
            in_timing_section = false;
        }
        
        bool should_parse = in_timing_section || !found_timing;
        
        if (should_parse) {
            if (line.find("\"dt_s\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    size_t comma = value_str.find(",");
                    if (comma != std::string::npos) {
                        value_str = value_str.substr(0, comma);
                    }
                    cfg.dt_s = std::stod(value_str);
                }
            }
            else if (line.find("\"duration_s\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    size_t comma = value_str.find(",");
                    if (comma != std::string::npos) {
                        value_str = value_str.substr(0, comma);
                    }
                    cfg.duration_s = std::stod(value_str);
                }
            }
            else if (line.find("\"log_hz\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    size_t comma = value_str.find(",");
                    if (comma != std::string::npos) {
                        value_str = value_str.substr(0, comma);
                    }
                    cfg.log_hz = std::stod(value_str);
                }
            }
            else if (line.find("\"real_time_mode\"") != std::string::npos) {
                if (line.find("true") != std::string::npos) {
                    cfg.real_time_mode = true;
                } else if (line.find("false") != std::string::npos) {
                    cfg.real_time_mode = false;
                }
            }
        }
    }
    
    return true;
}

// Parse vehicle_config field from JSON (if present)
std::string load_vehicle_config_path_from_json(const std::string& scenario_path) {
    std::ifstream file(scenario_path);
    if (!file.is_open()) {
        return "";
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("\"vehicle_config\"") != std::string::npos) {
            size_t colon = line.find(":");
            if (colon != std::string::npos) {
                std::string value_str = line.substr(colon + 1);
                value_str.erase(0, value_str.find_first_not_of(" \t\r\n\""));
                value_str.erase(value_str.find_last_not_of(" \t\r\n\",") + 1);
                return value_str;
            }
        }
    }
    
    return "";
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
    printf("  --help, -h            Show this help\n");
    
    printf("\nInfluxDB Options:\n");
    printf("  --influx              Enable InfluxDB time-series logging\n");
    printf("  --influx-url URL      InfluxDB server URL (default: http://localhost:8086)\n");
    printf("  --influx-token TOKEN  Authentication token (optional for local)\n");
    printf("  --influx-org ORG      Organization name (default: Autonomy)\n");
    printf("  --influx-bucket NAME  Bucket name (default: vehicle-sim)\n");
    printf("  --influx-interval MS  Write interval in ms (default: 250)\n");
    printf("\n  Note: InfluxDB logging requires --real-time mode\n");
    
    printf("\nExamples:\n");
    printf("  # Open-loop with scenario file:\n");
    printf("  %s config/scenarios/slalom.json\n\n", prog_name);
    
    printf("  # Real-time with InfluxDB logging:\n");
    printf("  %s --real-time --influx config/scenarios/slalom.json\n\n", prog_name);
    
    printf("  # Custom InfluxDB server with authentication:\n");
    printf("  %s --real-time --influx --influx-url http://192.168.1.100:8086 \\\n", prog_name);
    printf("    --influx-token \"mytoken123\" config/scenarios/slalom.json\n\n");
    
    printf("  # High-frequency InfluxDB logging (10Hz):\n");
    printf("  %s --real-time --influx --influx-interval 100 config/scenarios/slalom.json\n\n", prog_name);
    
    printf("  # Closed-loop with InfluxDB:\n");
    printf("  %s --can-rx --real-time --influx --duration 60\n\n", prog_name);
    
    printf("  # Closed-loop with faster timestep:\n");
    printf("  %s --can-rx --dt 0.001 --duration 30\n\n", prog_name);
    
    printf("  # Open-loop, fast-forward mode (no real-time):\n");
    printf("  %s --fast config/scenarios/brake_test.json\n\n", prog_name);
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
    
    cfg.use_lua_scenario = true;
    cfg.lua_script_path = "config/lua/scenario.lua";
    cfg.scenario_json_path = "config/scenarios/brake_test.json";
    
    cfg.csv_log_path = "sim_out.csv";
    cfg.debug_log_path = "sim_debug.log";
    cfg.enable_debug_log_file = true;

    cfg.enable_can_tx = true;
    cfg.enable_can_rx = false;
    cfg.can_interface = "vcan0";
    cfg.can_map_path = "config/can_map.csv";
    cfg.actuator_cmd_frame_name = "ACTUATOR_CMD_1";
    cfg.can_rx_timeout_s = 0.5;
    
    // InfluxDB defaults
    cfg.enable_influx = false;
    cfg.influx_url = "http://localhost:8086";
    cfg.influx_token = "";  // Empty = no auth (local only)
    cfg.influx_org = "Autonomy";
    cfg.influx_bucket = "vehicle-sim";
    cfg.influx_interval_s = 0.25;  // 250ms = 4Hz
    
    std::string vehicle_config_path = "";
    
    // ========================================================================
    // Command-line parsing
    // ========================================================================
    static struct option long_options[] = {
        {"can-rx",          no_argument,       0, 'r'},
        {"can-tx",          no_argument,       0, 't'},
        {"no-can-tx",       no_argument,       0, 'T'},
        {"can-iface",       required_argument, 0, 'i'},
        {"can-timeout",     required_argument, 0, 'x'},
        {"real-time",       no_argument,       0, 'R'},
        {"fast",            no_argument,       0, 'F'},
        {"dt",              required_argument, 0, 'd'},
        {"duration",        required_argument, 0, 'D'},
        {"vehicle",         required_argument, 0, 'v'},
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
                cfg.use_lua_scenario = false;
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
            case 'v':
                vehicle_config_path = optarg;
                break;
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
    
    if (!cfg.enable_can_rx && optind < argc) {
        cfg.scenario_json_path = argv[optind];
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

    utils::set_level(utils::LogLevel::Info);

    // ========================================================================
    // Load timing and vehicle configuration
    // ========================================================================
    if (!cfg.enable_can_rx) {
        load_timing_from_json(cfg.scenario_json_path, cfg);
        LOG_INFO("Loaded config from JSON: dt=%.4fs, duration=%.1fs, log_hz=%.1f, realtime=%s",
                 cfg.dt_s, cfg.duration_s, cfg.log_hz, 
                 cfg.real_time_mode ? "true" : "false");
    }

    config::VehicleConfig vehicle;
    
    if (!vehicle_config_path.empty()) {
        LOG_INFO("Loading vehicle from command-line: %s", vehicle_config_path.c_str());
        vehicle = config::VehicleConfig::load(vehicle_config_path);
    } else {
        std::string json_vehicle_path = load_vehicle_config_path_from_json(cfg.scenario_json_path);
        if (!json_vehicle_path.empty()) {
            LOG_INFO("Loading vehicle from JSON: %s", json_vehicle_path.c_str());
            vehicle = config::VehicleConfig::load(json_vehicle_path);
        } else {
            LOG_INFO("No vehicle config specified, using defaults");
            vehicle = config::VehicleConfig::get_default();
        }
    }
    
    vehicle.print_summary();
    cfg.vehicle_params = vehicle.params;

    // ========================================================================
    // Print configuration summary
    // ========================================================================
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║       PLANT-SENSOR-CAN SIMULATION CONFIGURATION            ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║ Mode:       %-48s║\n", cfg.enable_can_rx ? "CLOSED-LOOP (CAN RX)" : "OPEN-LOOP (Scenario)");
    printf("║ CAN TX:     %-48s║\n", cfg.enable_can_tx ? "enabled" : "disabled");
    printf("║ CAN RX:     %-48s║\n", cfg.enable_can_rx ? "enabled" : "disabled");
    printf("║ Interface:  %-48s║\n", cfg.can_interface.c_str());
    if (cfg.enable_can_rx) {
        char timeout_str[50];
        snprintf(timeout_str, sizeof(timeout_str), "%.2f seconds", cfg.can_rx_timeout_s);
        printf("║ RX Timeout: %-48s║\n", timeout_str);
        printf("║ RX Frame:   %-48s║\n", cfg.actuator_cmd_frame_name.c_str());
    }
    if (!cfg.enable_can_rx) {
        printf("║ Scenario:   %-48s║\n", cfg.scenario_json_path.c_str());
    }
    char timestep_str[50], duration_str[50];
    snprintf(timestep_str, sizeof(timestep_str), "%.4f seconds", cfg.dt_s);
    snprintf(duration_str, sizeof(duration_str), "%.1f seconds", cfg.duration_s);
    printf("║ Timestep:   %-48s║\n", timestep_str);
    printf("║ Duration:   %-48s║\n", duration_str);
    printf("║ Real-time:  %-48s║\n", cfg.real_time_mode ? "yes (1:1 wall clock)" : "no (fast-forward)");
    printf("║ Vehicle:    %-48s║\n", vehicle.name.c_str());
    
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
        printf("    Frame: %s (ID: 0x100)\n", cfg.actuator_cmd_frame_name.c_str());
        printf("    Start your controller now!\n\n");
    }
    
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
    if (!cfg.enable_can_rx) {
        LOG_INFO("Scenario: %s", cfg.scenario_json_path.c_str());
    }
    LOG_INFO("Vehicle: %s", vehicle.name.c_str());
    LOG_INFO("========================================");

    sim::SimApp app(cfg);
    return app.run_plant_only();
}