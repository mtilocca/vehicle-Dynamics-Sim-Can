// src/sim/sim_main.cpp
#include "sim/sim_app.hpp"
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <cstring>
#include <getopt.h>

static bool parse_log_level(const char* str, utils::LogLevel& out) {
    if (strcasecmp(str, "trace") == 0) { out = utils::LogLevel::Trace; return true; }
    if (strcasecmp(str, "debug") == 0) { out = utils::LogLevel::Debug; return true; }
    if (strcasecmp(str, "info")  == 0) { out = utils::LogLevel::Info;  return true; }
    if (strcasecmp(str, "warn")  == 0) { out = utils::LogLevel::Warn;  return true; }
    if (strcasecmp(str, "error") == 0) { out = utils::LogLevel::Error; return true; }
    if (strcasecmp(str, "off")   == 0) { out = utils::LogLevel::Off;   return true; }
    return false;
}

static void print_usage(const char* prog) {
    printf("Usage: %s [options]\n\n", prog);
    printf("Runs a 3-DOF vehicle plant with CAN closed-loop control.\n");
    printf("The plant publishes state frames on CAN and reads actuator commands from CAN.\n\n");

    printf("Options:\n");
    printf("  --can-rx              Enable CAN RX (closed-loop). Default: off (zero cmd)\n");
    printf("  --no-can-tx           Disable CAN TX of plant-state frames\n");
    printf("  --can-iface NAME      CAN interface (default: vcan0)\n");
    printf("  --can-map PATH        Signal map CSV (default: config/can_map.csv)\n");
    printf("  --can-timeout SEC     CAN RX watchdog timeout (default: 0.5 s)\n");
    printf("  --real-time           Pace simulation to wall clock (required for InfluxDB)\n");
    printf("  --fast                Fast-forward mode (no real-time pacing)\n");
    printf("  --dt SEC              Integration timestep (default: 0.01 s)\n");
    printf("  --duration SEC        Run duration, 0 = indefinite (default: 0)\n");
    printf("  --log-hz HZ           CSV log rate (default: 10 Hz)\n");
    printf("  --vehicle PATH        Vehicle config YAML (default: built-in heavy truck)\n");
    printf("  --surface-mu MU       Surface friction coefficient (default: 0.72)\n");
    printf("                          0.85 dry pavement | 0.72 compact gravel\n");
    printf("                          0.55 loose gravel | 0.30 wet mud\n");
    printf("  --log-level LEVEL     trace|debug|info|warn|error|off (default: info)\n");
    printf("\nInfluxDB options (require --real-time):\n");
    printf("  --influx              Enable InfluxDB logging\n");
    printf("  --influx-url URL      (default: http://localhost:8086)\n");
    printf("  --influx-token TOKEN  Bearer token for authentication\n");
    printf("  --influx-org ORG      (default: Autonomy)\n");
    printf("  --influx-bucket NAME  (default: vehicle-sim)\n");
    printf("  --influx-interval MS  Write interval in ms (default: 250)\n");
    printf("\nExamples:\n");
    printf("  # Open-loop (zero command, fast-forward 30 s):\n");
    printf("  %s --fast --duration 30\n\n", prog);
    printf("  # Closed-loop on virtual CAN:\n");
    printf("  %s --can-rx --real-time --duration 60 --can-iface vcan0\n\n", prog);
    printf("  # With InfluxDB and custom vehicle:\n");
    printf("  %s --can-rx --real-time --influx --vehicle config/vehicles/heavy_truck.yaml\n\n", prog);
}

int main(int argc, char** argv) {
    sim::SimAppConfig cfg{};
    cfg.dt_s       = 0.01;
    cfg.duration_s = 0.0;   // indefinite by default
    cfg.log_hz     = 10.0;
    cfg.real_time_mode = true;

    cfg.csv_log_path        = "sim_out.csv";
    cfg.debug_log_path      = "sim_debug.log";
    cfg.enable_debug_log_file = true;

    cfg.enable_can_tx           = true;
    cfg.enable_can_rx           = false;
    cfg.can_interface           = "vcan0";
    cfg.can_map_path            = "config/can_map.csv";
    cfg.actuator_cmd_frame_name = "ACTUATOR_CMD_1";
    cfg.can_rx_timeout_s        = 0.5;

    cfg.surface_friction = 0.72;

    cfg.enable_influx    = false;
    cfg.influx_url       = "http://localhost:8086";
    cfg.influx_token     = "";
    cfg.influx_org       = "Autonomy";
    cfg.influx_bucket    = "vehicle-sim";
    cfg.influx_interval_s = 0.25;

    std::string vehicle_config_path;
    utils::LogLevel log_level = utils::LogLevel::Info;

    // =========================================================================
    // Command-line parsing
    // =========================================================================
    static struct option long_options[] = {
        {"can-rx",          no_argument,       0, 'r'},
        {"no-can-tx",       no_argument,       0, 'T'},
        {"can-iface",       required_argument, 0, 'i'},
        {"can-map",         required_argument, 0, 'M'},
        {"can-timeout",     required_argument, 0, 'x'},
        {"real-time",       no_argument,       0, 'R'},
        {"fast",            no_argument,       0, 'F'},
        {"dt",              required_argument, 0, 'd'},
        {"duration",        required_argument, 0, 'D'},
        {"log-hz",          required_argument, 0, 'z'},
        {"vehicle",         required_argument, 0, 'v'},
        {"surface-mu",      required_argument, 0, 'm'},
        {"log-level",       required_argument, 0, 'L'},
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
    int opt_idx = 0;
    while ((opt = getopt_long(argc, argv, "h", long_options, &opt_idx)) != -1) {
        switch (opt) {
            case 'r':  cfg.enable_can_rx = true;                              break;
            case 'T':  cfg.enable_can_tx = false;                             break;
            case 'i':  cfg.can_interface = optarg;                            break;
            case 'M':  cfg.can_map_path  = optarg;                            break;
            case 'x':
                cfg.can_rx_timeout_s = std::atof(optarg);
                if (cfg.can_rx_timeout_s <= 0) {
                    fprintf(stderr, "Error: invalid --can-timeout: %s\n", optarg);
                    return 1;
                }
                break;
            case 'R':  cfg.real_time_mode = true;                             break;
            case 'F':  cfg.real_time_mode = false;                            break;
            case 'd':
                cfg.dt_s = std::atof(optarg);
                if (cfg.dt_s <= 0 || cfg.dt_s > 1.0) {
                    fprintf(stderr, "Error: invalid --dt: %s\n", optarg);
                    return 1;
                }
                break;
            case 'D':
                cfg.duration_s = std::atof(optarg);
                if (cfg.duration_s < 0) {
                    fprintf(stderr, "Error: invalid --duration: %s\n", optarg);
                    return 1;
                }
                break;
            case 'z':
                cfg.log_hz = std::atof(optarg);
                if (cfg.log_hz <= 0) {
                    fprintf(stderr, "Error: invalid --log-hz: %s\n", optarg);
                    return 1;
                }
                break;
            case 'v':  vehicle_config_path = optarg;                          break;
            case 'm':
                cfg.surface_friction = std::atof(optarg);
                if (cfg.surface_friction <= 0 || cfg.surface_friction > 1.5) {
                    fprintf(stderr, "Error: invalid --surface-mu: %s\n", optarg);
                    return 1;
                }
                break;
            case 'L':
                if (!parse_log_level(optarg, log_level)) {
                    fprintf(stderr, "Error: invalid --log-level: %s\n", optarg);
                    return 1;
                }
                break;
            case 'I':  cfg.enable_influx = true;                              break;
            case 'U':  cfg.influx_url    = optarg;                            break;
            case 'K':  cfg.influx_token  = optarg;                            break;
            case 'O':  cfg.influx_org    = optarg;                            break;
            case 'B':  cfg.influx_bucket = optarg;                            break;
            case 'N':
                cfg.influx_interval_s = std::atof(optarg) / 1000.0;
                if (cfg.influx_interval_s <= 0) {
                    fprintf(stderr, "Error: invalid --influx-interval: %s\n", optarg);
                    return 1;
                }
                break;
            case 'h':  print_usage(argv[0]); return 0;
            default:   print_usage(argv[0]); return 1;
        }
    }

    if (cfg.enable_influx && !cfg.real_time_mode) {
        fprintf(stderr, "Warning: InfluxDB requires --real-time mode. Disabling InfluxDB.\n");
        cfg.enable_influx = false;
    }

    utils::set_level(log_level);

    // =========================================================================
    // Load vehicle configuration
    // =========================================================================
    config::VehicleConfig vehicle;

    if (!vehicle_config_path.empty()) {
        LOG_INFO("[Main] Loading vehicle: %s", vehicle_config_path.c_str());
        vehicle = config::VehicleConfig::load(vehicle_config_path);
    } else {
        LOG_INFO("[Main] No vehicle config specified — using built-in defaults");
        vehicle = config::VehicleConfig::get_default();
    }

    vehicle.print_summary();
    cfg.vehicle_params = vehicle.params;

    // =========================================================================
    // Print startup banner
    // =========================================================================
    printf("\n");
    printf("====================================================\n");
    printf("  Vehicle Dynamics Simulation — 3-DOF Plant\n");
    printf("====================================================\n");
    printf("  Mode:       %s\n", cfg.enable_can_rx ? "CLOSED-LOOP (CAN RX)" : "OPEN-LOOP (zero cmd)");
    printf("  CAN TX:     %s\n", cfg.enable_can_tx ? "enabled" : "disabled");
    printf("  CAN RX:     %s\n", cfg.enable_can_rx ? "enabled" : "disabled");
    printf("  Interface:  %s\n", cfg.can_interface.c_str());
    printf("  dt:         %.4f s\n", cfg.dt_s);
    printf("  Duration:   %.1f s%s\n", cfg.duration_s,
           cfg.duration_s == 0.0 ? " (indefinite)" : "");
    printf("  Real-time:  %s\n", cfg.real_time_mode ? "yes" : "no (fast-forward)");
    printf("  Surface mu: %.2f\n", cfg.surface_friction);
    printf("  Vehicle:    %s\n", vehicle.name.c_str());
    printf("  InfluxDB:   %s\n", cfg.enable_influx ? "enabled" : "disabled");
    printf("====================================================\n\n");

    if (cfg.enable_can_rx) {
        printf("Waiting for CAN commands on %s (frame %s / 0x100).\n",
               cfg.can_interface.c_str(), cfg.actuator_cmd_frame_name.c_str());
        printf("Start your controller now.\n\n");
    }

    // =========================================================================
    // Run
    // =========================================================================
    sim::SimApp app(cfg);
    return app.run_plant();
}
