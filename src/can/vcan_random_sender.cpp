#include <csignal>
#include <random>
#include <string>

#include "can/can_codec.hpp"
#include "can/can_map.hpp"
#include "can/socketcan_iface.hpp"
#include "can/tx_scheduler.hpp"
#include "utils/logging.hpp"

static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

static bool parse_u32(const char* s, uint32_t& out) {
    char* end = nullptr;
    errno = 0;
    unsigned long v = std::strtoul(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0') return false;
    out = static_cast<uint32_t>(v);
    return true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sigint);
    utils::set_level(utils::LogLevel::Info);

    const char* ifname   = (argc > 1) ? argv[1] : "vcan0";
    const char* csv_path = (argc > 2) ? argv[2] : "config/can_map.dbc";

    // Optional: restrict to a single TX frame (J1939 29-bit ID), for debugging.
    // Pass the raw 29-bit ID (without the EFF flag), e.g.:
    //   ./vcan_random_sender vcan0 config/can_map.dbc 0x0CFF0028   (IMU_ACC)
    bool use_fixed_id = false;
    uint32_t fixed_id = 0;
    if (argc > 3) {
        if (!parse_u32(argv[3], fixed_id)) {
            LOG_ERROR("Invalid fixed_id. Use J1939 29-bit ID, e.g. 0x0CFF0028");
            return 1;
        }
        use_fixed_id = true;
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

    // Prepare list of TX frames we will send
    std::vector<can::FrameDef> tx_frames;
    tx_frames.reserve(map.tx_frames().size());

    for (const auto& f : map.tx_frames()) {
        // Compare raw 29-bit part (strip EFF flag from stored frame_id)
        if (!use_fixed_id || (f.frame_id & CAN_EFF_MASK) == (fixed_id & CAN_EFF_MASK))
            tx_frames.push_back(f);
    }

    if (tx_frames.empty()) {
        LOG_ERROR("No TX frames selected. Check CSV or fixed_id filter.");
        return 1;
    }

    can::TxScheduler sched;
    sched.init(tx_frames);
    sched.force_all_due();

    std::mt19937 rng{std::random_device{}()};

    LOG_INFO("CSV-driven CAN random sender on %s", ifname);
    LOG_INFO("Map: %s", csv_path);
    if (use_fixed_id) LOG_INFO("Fixed TX frame: 0x%08X", fixed_id);

    while (!g_stop) {
        auto now = can::TxScheduler::Clock::now();
        auto due = sched.due(now);

        for (size_t idx : due) {
            const auto& def = tx_frames[idx];

            // Build engineering values for this frame
            can::SignalMap vals;
            vals.reserve(def.signals.size());

            for (const auto& sig : def.signals) {
                // If min/max are equal or reversed, just use default
                double lo = sig.min;
                double hi = sig.max;

                double eng = sig.default_value;
                if (hi > lo) {
                    std::uniform_real_distribution<double> dist(lo, hi);
                    eng = dist(rng);
                }

                vals[sig.signal_name] = eng;
            }

            // Encode -> CAN frame
            struct can_frame frame{};
            can::CanCodec::encode_from_map(def, vals, frame);

            // Send
            if (!iface.write_frame(frame)) {
                LOG_ERROR("Failed to write frame 0x%08X", def.frame_id);
                g_stop = 1;
                break;
            }

            LOG_INFO("TX 0x%08X (%s)", def.frame_id, def.frame_name.c_str());
        }
    }

    LOG_INFO("Stopping TX");
    return 0;
}