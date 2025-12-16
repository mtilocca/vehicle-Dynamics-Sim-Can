#include <chrono>
#include <csignal>
#include <cstring>
#include <random>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "can/can_map.hpp"
#include "utils/bitpack.hpp"
#include "utils/logging.hpp"

using namespace std::chrono;

static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

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
    std::signal(SIGINT, on_sigint);
    utils::set_level(utils::LogLevel::Info);

    const char* ifname = (argc > 1) ? argv[1] : "vcan0";
    const char* csv_path = (argc > 2) ? argv[2] : "configs/can_map.csv";

    can::CanMap map;
    if (!map.load(csv_path)) {
        LOG_ERROR("Failed to load CAN map: %s", csv_path);
        return 1;
    }

    int sock = open_can_socket(ifname);
    if (sock < 0) {
        LOG_ERROR("Failed to open CAN socket");
        return 1;
    }

    std::mt19937 rng{std::random_device{}()};
    LOG_INFO("CSV-driven CAN TX on %s", ifname);

    // Per-frame next send time
    std::vector<steady_clock::time_point> next_tx;
    next_tx.resize(map.tx_frames().size(), steady_clock::now());

    while (!g_stop)
    {
        auto now = steady_clock::now();

        for (size_t i = 0; i < map.tx_frames().size(); ++i) {
            const auto& frame_def = map.tx_frames()[i];
            if (now < next_tx[i]) continue;

            struct can_frame frame{};
            frame.can_id = frame_def.frame_id;
            frame.can_dlc = frame_def.dlc;
            std::memset(frame.data, 0, sizeof(frame.data));

            for (const auto& sig : frame_def.signals) {
                std::uniform_real_distribution<double> dist(sig.min, sig.max);
                double eng = dist(rng);
                double raw_f = (eng - sig.offset) / sig.factor;
                int64_t raw = sig.is_signed ?
                    static_cast<int64_t>(raw_f) :
                    static_cast<uint64_t>(raw_f);

                utils::set_bits_lsb0(
                    frame.data, frame.can_dlc,
                    sig.start_bit, sig.bit_length,
                    sig.endianness,
                    static_cast<uint64_t>(raw)
                );
            }

            ::write(sock, &frame, sizeof(frame));

            LOG_INFO("TX 0x%03X (%s)", frame_def.frame_id,
                     frame_def.frame_name.c_str());

            next_tx[i] = now + milliseconds(frame_def.cycle_ms);
        }

        std::this_thread::sleep_for(milliseconds(1));
    }

    LOG_INFO("Stopping TX");
    ::close(sock);
    return 0;
}