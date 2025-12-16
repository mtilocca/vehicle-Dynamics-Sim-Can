#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "can/can_map.hpp"
#include "utils/bitpack.hpp"
#include "utils/logging.hpp"

static int open_can_socket(const char *ifname)
{
    int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket(PF_CAN)");
        return -1;
    }

    struct ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        ::close(s);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind(AF_CAN)");
        ::close(s);
        return -1;
    }

    return s;
}

static bool arg_is_true(const std::string& s) {
    // Accept: 1/0, true/false, yes/no, on/off
    auto lower = s;
    for (auto& c : lower) c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));
    return (lower == "1" || lower == "true" || lower == "yes" || lower == "on");
}

int main(int argc, char **argv)
{
    utils::set_level(utils::LogLevel::Info);

    const char* ifname   = (argc > 1) ? argv[1] : "vcan0";
    const char* csv_path = (argc > 2) ? argv[2] : "configs/can_map.csv";

    // Optional: decode TX frames too (default: OFF)
    // Usage:
    //   ./vcan_listener vcan0 configs/can_map.csv
    //   ./vcan_listener vcan0 configs/can_map.csv --decode-tx
    //   ./vcan_listener vcan0 configs/can_map.csv --decode-tx=1
    bool decode_tx = false;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--decode-tx") decode_tx = true;
        else if (a.rfind("--decode-tx=", 0) == 0) decode_tx = arg_is_true(a.substr(std::string("--decode-tx=").size()));
    }

    can::CanMap map;
    if (!map.load(csv_path)) {
        LOG_ERROR("Failed to load CAN map: %s", csv_path);
        return 1;
    }

    int sock = open_can_socket(ifname);
    if (sock < 0) return 1;

    LOG_INFO("Listening on %s", ifname);
    LOG_INFO("Decode TX frames: %s", decode_tx ? "ON" : "OFF");

    while (true)
    {
        struct can_frame frame{};
        ssize_t n = ::read(sock, &frame, sizeof(frame));
        if (n < 0) {
            perror("read(can_frame)");
            break;
        }
        if (n != (ssize_t)sizeof(frame)) continue;

        uint32_t id = frame.can_id & CAN_SFF_MASK;

        const can::FrameDef* def = map.find_rx_frame(id);
        if (!def && decode_tx) def = map.find_tx_frame(id);
        if (!def) continue;

        LOG_INFO("RX 0x%03X (%s) dlc=%d", id, def->frame_name.c_str(), (int)frame.can_dlc);

        for (const auto& sig : def->signals) {
            uint64_t raw_u = utils::get_bits_lsb0(
                frame.data, frame.can_dlc,
                sig.start_bit, sig.bit_length,
                sig.endianness
            );

            double eng = 0.0;
            if (sig.is_signed) {
                int64_t raw_s = utils::sign_extend(raw_u, sig.bit_length);
                eng = (double)raw_s * sig.factor + sig.offset;
            } else {
                eng = (double)raw_u * sig.factor + sig.offset;
            }

            LOG_INFO("  %-24s = %.6f %s", sig.signal_name.c_str(), eng, sig.unit.c_str());
        }
    }

    ::close(sock);
    return 0;
}