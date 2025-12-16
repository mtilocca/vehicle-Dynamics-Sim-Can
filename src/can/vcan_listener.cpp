#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "can_map.hpp"
#include "utils/bitpack.hpp"
#include "utils/logging.hpp"

static int open_can_socket(const char *ifname)
{
    int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;

    struct ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0) return -1;

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) return -1;
    return s;
}

int main(int argc, char **argv)
{
    utils::set_level(utils::LogLevel::Info);

    const char* ifname = (argc > 1) ? argv[1] : "vcan0";
    const char* csv_path = (argc > 2) ? argv[2] : "configs/can_map.csv";

    can::CanMap map;
    if (!map.load(csv_path)) {
        LOG_ERROR("Failed to load CAN map");
        return 1;
    }

    int sock = open_can_socket(ifname);
    if (sock < 0) {
        LOG_ERROR("Failed to open CAN socket");
        return 1;
    }

    LOG_INFO("Listening on %s", ifname);

    while (true)
    {
        struct can_frame frame{};
        ssize_t n = ::read(sock, &frame, sizeof(frame));
        if (n != sizeof(frame)) continue;

        uint32_t id = frame.can_id & CAN_SFF_MASK;
        const auto* frame_def = map.find_rx_frame(id);
        if (!frame_def) continue;

        LOG_INFO("RX 0x%03X (%s)", id, frame_def->frame_name.c_str());

        for (const auto& sig : frame_def->signals) {
            uint64_t raw_u =
                utils::get_bits_lsb0(frame.data, frame.can_dlc,
                                     sig.start_bit, sig.bit_length,
                                     sig.endianness);

            double eng;
            if (sig.is_signed) {
                int64_t raw_s = utils::sign_extend(raw_u, sig.bit_length);
                eng = raw_s * sig.factor + sig.offset;
            } else {
                eng = raw_u * sig.factor + sig.offset;
            }

            LOG_INFO("  %-24s = %.3f %s",
                     sig.signal_name.c_str(),
                     eng,
                     sig.unit.c_str());
        }
    }

    ::close(sock);
    return 0;
}