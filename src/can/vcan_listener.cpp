// src/can/vcan_listener.cpp
#include <cctype>
#include <string>
#include <vector>
#include <map>
#include <ctime>
#include <cstdio>
#include <thread>
#include <chrono>

#include <linux/can.h>

#include "can/can_codec.hpp"
#include "can/can_map.hpp"
#include "can/socketcan_iface.hpp"
#include "utils/logging.hpp"

static bool arg_is_true(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return (s == "1" ||  s == "true" || s == "yes" || s == "on");
}

static bool parse_u32(const std::string& s, uint32_t& out) {
    char* end = nullptr;
    errno = 0;
    unsigned long v = std::strtoul(s.c_str(), &end, 0);
    if (errno != 0 || end == s.c_str() || *end != '\0') return false;
    out = static_cast<uint32_t>(v);
    return true;
}

static std::vector<uint32_t> parse_id_list(const std::string& s) {
    // e.g. "0x200,0x201,0x220"
    std::vector<uint32_t> ids;
    std::string cur;
    for (char ch : s) {
        if (ch == ',') {
            if (!cur.empty()) {
                uint32_t v = 0;
                if (parse_u32(cur, v) && v <= 0x7FF) ids.push_back(v);
                cur.clear();
            }
        } else if (!std::isspace(static_cast<unsigned char>(ch))) {
            cur.push_back(ch);
        }
    }
    if (!cur.empty()) {
        uint32_t v = 0;
        if (parse_u32(cur, v) && v <= 0x7FF) ids.push_back(v);
    }
    return ids;
}

// Live monitoring state tracker
struct LiveMonitor {
    std::map<std::string, double> latest_values;
    uint64_t frame_count = 0;
    std::time_t start_time;
    
    LiveMonitor() {
        start_time = std::time(nullptr);
    }
    
    void update(const std::string& signal, double value) {
        latest_values[signal] = value;
    }
    
    void print_dashboard() {
        std::printf("\033[2J\033[H");  // Clear screen and move cursor to top
        
        std::time_t now = std::time(nullptr);
        double elapsed = std::difftime(now, start_time);
        
        std::printf("╔════════════════════════════════════════════════════════════════════════════╗\n");
        std::printf("║               PLANT STATE LIVE MONITOR - vCAN Listener                    ║\n");
        std::printf("╚════════════════════════════════════════════════════════════════════════════╝\n");
        std::printf("  Frames: %llu | Elapsed: %.0f s\n\n", 
                    (unsigned long long)frame_count, elapsed);
        
        // Vehicle State
        std::printf("┌─ VEHICLE STATE ────────────────────────────────────────────────────────────┐\n");
        print_value("Speed", "vehicle_speed_mps", "m/s", 8, 2);
        print_value("Accel", "vehicle_accel_mps2", "m/s²", 8, 2);
        print_value("Yaw Rate", "yaw_rate_radps", "rad/s", 8, 3);
        std::printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");
        
        // Motor State
        std::printf("┌─ MOTOR STATE ──────────────────────────────────────────────────────────────┐\n");
        print_value("Torque", "motor_torque_nm", "Nm", 8, 1);
        print_value("Power", "motor_power_kw", "kW", 8, 1);
        print_value("RPM", "motor_speed_rpm", "rpm", 8, 0);
        print_value("Temp", "motor_temp_c", "°C", 8, 1);
        std::printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");
        
        // Brake State
        std::printf("┌─ BRAKE STATE ──────────────────────────────────────────────────────────────┐\n");
        print_value("Force", "brake_force_kn", "kN", 8, 2);
        print_value("Applied", "brake_pct_actual", "%", 8, 1);
        print_value("Regen Power", "regen_power_kw", "kW", 8, 1);
        print_value("Temp", "brake_temp_c", "°C", 8, 1);
        std::printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");
        
        // Position & Orientation
        std::printf("┌─ POSITION & ORIENTATION ───────────────────────────────────────────────────┐\n");
        print_value("X Position", "pos_x_m", "m", 10, 2);
        print_value("Y Position", "pos_y_m", "m", 10, 2);
        print_value("Yaw", "yaw_deg", "deg", 8, 2);
        print_value("Yaw Rate", "yaw_rate_dps", "deg/s", 8, 2);
        std::printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");
        
        // Battery State (from 0x230)
        std::printf("┌─ BATTERY STATE ────────────────────────────────────────────────────────────┐\n");
        print_value("SOC", "batt_soc_pct", "%", 8, 1);
        print_value("Voltage", "batt_v", "V", 8, 1);
        print_value("Current", "batt_i", "A", 8, 1);
        print_value("Power", "batt_power_kw", "kW", 8, 1);
        std::printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");
        
        // Simulation Time
        std::printf("┌─ SIMULATION ───────────────────────────────────────────────────────────────┐\n");
        print_value("Sim Time", "sim_time_s", "s", 8, 2);
        print_value("Loop Time", "loop_time_us", "µs", 8, 0);
        std::printf("└────────────────────────────────────────────────────────────────────────────┘\n");
        
        std::printf("\n[Press Ctrl+C to exit]\n");
        std::fflush(stdout);
    }
    
private:
    void print_value(const char* label, const char* signal, const char* unit, 
                     int width, int precision) {
        auto it = latest_values.find(signal);
        if (it != latest_values.end()) {
            std::printf("  %-20s: %*.*f %-10s", label, width, precision, it->second, unit);
        } else {
            std::printf("  %-20s: %*s %-10s", label, width, "---", unit);
        }
        std::printf("\n");
    }
};

int main(int argc, char** argv) {
    utils::set_level(utils::LogLevel::Info);

    const char* ifname   = (argc > 1) ? argv[1] : "vcan0";
    const char* csv_path = (argc > 2) ? argv[2] : "config/can_map.csv";

    // Flags:
    //   --decode-tx           (enable TX decoding)
    //   --decode-tx=1|0       (explicit)
    //   --filter=0x200,0x201  (optional SocketCAN filter)
    //   --live                (live dashboard mode - NEW!)
    //   --plant-only          (filter only plant_state frames - NEW!)
    bool decode_tx = false;
    bool live_mode = false;
    bool plant_only = false;
    std::vector<uint32_t> filter_ids;

    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--decode-tx") {
            decode_tx = true;
        } else if (a.rfind("--decode-tx=", 0) == 0) {
            decode_tx = arg_is_true(a.substr(std::string("--decode-tx=").size()));
        } else if (a.rfind("--filter=", 0) == 0) {
            filter_ids = parse_id_list(a.substr(std::string("--filter=").size()));
        } else if (a == "--live") {
            live_mode = true;
            decode_tx = true;  // Live mode requires TX decoding
        } else if (a == "--plant-only") {
            plant_only = true;
            decode_tx = true;  // Plant-only requires TX decoding
        }
    }

    can::CanMap map;
    if (!map.load(csv_path)) {
        LOG_ERROR("Failed to load CAN map: %s", csv_path);
        return 1;
    }

    can::SocketCanIface iface;
    if (!iface.open(ifname)) {
        LOG_ERROR("Failed to open SocketCAN iface: %s", ifname);
        return 1;
    }

    // Auto-filter to plant_state frames if --plant-only
    if (plant_only && filter_ids.empty()) {
        // Plant state frame IDs: 0x300, 0x310, 0x320, 0x330, 0x331, 0x340, 0x3F0
        filter_ids = {0x300, 0x310, 0x320, 0x330, 0x331, 0x340, 0x3F0};
    }

    if (!filter_ids.empty()) {
        if (!iface.set_filters(filter_ids)) {
            LOG_ERROR("Failed to set CAN filters");
            return 1;
        }
        LOG_INFO("Filter enabled (%zu IDs)", filter_ids.size());
    }

    LOG_INFO("Listening on %s", ifname);
    LOG_INFO("Map: %s", csv_path);
    LOG_INFO("Decode TX frames: %s", decode_tx ? "ON" : "OFF");
    if (live_mode) {
        LOG_INFO("Live dashboard mode: ENABLED");
        std::printf("\nStarting live monitor in 2 seconds...\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    LiveMonitor monitor;
    uint64_t update_counter = 0;
    const uint64_t update_interval = live_mode ? 10 : 0;  // Update every 10 frames in live mode

    while (true) {
        struct can_frame frame{};
        if (!iface.read_frame(frame)) continue;

        uint32_t id = frame.can_id & CAN_SFF_MASK;

        const can::FrameDef* def = map.find_rx_frame(id);
        if (!def && decode_tx) def = map.find_tx_frame(id);
        if (!def) continue;

        auto decoded = can::CanCodec::decode_to_map(*def, frame);

        if (live_mode) {
            // Update monitor state
            for (const auto& sig : def->signals) {
                auto it = decoded.find(sig.signal_name);
                if (it != decoded.end()) {
                    monitor.update(sig.signal_name, it->second);
                }
            }
            
            monitor.frame_count++;
            update_counter++;
            
            // Refresh dashboard periodically
            if (update_counter >= update_interval) {
                monitor.print_dashboard();
                update_counter = 0;
            }
        } else {
            // Normal logging mode
            LOG_INFO("RX 0x%03X (%s) dlc=%d", id, def->frame_name.c_str(), (int)frame.can_dlc);

            for (const auto& sig : def->signals) {
                auto it = decoded.find(sig.signal_name);
                if (it == decoded.end()) continue;

                LOG_INFO("  %-24s = %.6f %s",
                         sig.signal_name.c_str(),
                         it->second,
                         sig.unit.c_str());
            }
        }
    }

    return 0;
}