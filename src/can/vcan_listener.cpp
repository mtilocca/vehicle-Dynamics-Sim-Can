#include <cctype>
#include <string>
#include <vector>

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

int main(int argc, char** argv) {
    utils::set_level(utils::LogLevel::Info);

    const char* ifname   = (argc > 1) ? argv[1] : "vcan0";
    const char* csv_path = (argc > 2) ? argv[2] : "configs/can_map.csv";

    // Flags:
    //   --decode-tx           (enable)
    //   --decode-tx=1|0       (explicit)
    //   --filter=0x200,0x201  (optional SocketCAN filter)
    bool decode_tx = false;
    std::vector<uint32_t> filter_ids;

    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--decode-tx") {
            decode_tx = true;
        } else if (a.rfind("--decode-tx=", 0) == 0) {
            decode_tx = arg_is_true(a.substr(std::string("--decode-tx=").size()));
        } else if (a.rfind("--filter=", 0) == 0) {
            filter_ids = parse_id_list(a.substr(std::string("--filter=").size()));
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

    while (true) {
        struct can_frame frame{};
        if (!iface.read_frame(frame)) continue;

        uint32_t id = frame.can_id & CAN_SFF_MASK;

        const can::FrameDef* def = map.find_rx_frame(id);
        if (!def && decode_tx) def = map.find_tx_frame(id);
        if (!def) continue;

        LOG_INFO("RX 0x%03X (%s) dlc=%d", id, def->frame_name.c_str(), (int)frame.can_dlc);

        auto decoded = can::CanCodec::decode_to_map(*def, frame);

        for (const auto& sig : def->signals) {
            auto it = decoded.find(sig.signal_name);
            if (it == decoded.end()) continue;

            LOG_INFO("  %-24s = %.6f %s",
                     sig.signal_name.c_str(),
                     it->second,
                     sig.unit.c_str());
        }
    }

    return 0;
}