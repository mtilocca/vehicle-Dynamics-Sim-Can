#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

static volatile std::sig_atomic_t g_stop = 0;

static void on_sigint(int) { g_stop = 1; }

static int open_can_socket(const char* ifname) {
    int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket(PF_CAN)");
        return -1;
    }

    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        ::close(s);
        return -1;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind(AF_CAN)");
        ::close(s);
        return -1;
    }

    return s;
}

static bool parse_u32_hex(const char* s, uint32_t& out) {
    // Accept "123" or "0x123"
    char* end = nullptr;
    errno = 0;
    unsigned long v = std::strtoul(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0') return false;
    out = static_cast<uint32_t>(v);
    return true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sigint);

    const char* ifname = (argc > 1) ? argv[1] : "vcan0";
    int period_ms = (argc > 2) ? std::atoi(argv[2]) : 200;

    bool use_fixed_id = false;
    uint32_t fixed_id = 0;

    if (argc > 3) {
        if (!parse_u32_hex(argv[3], fixed_id) || fixed_id > 0x7FF) {
            std::cerr << "Invalid CAN ID. Use 11-bit ID (0..0x7FF), e.g. 123 or 0x123\n";
            return 1;
        }
        use_fixed_id = true;
    }

    if (period_ms < 0) period_ms = 0;

    int sock = open_can_socket(ifname);
    if (sock < 0) return 1;

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<uint32_t> id_dist(0x100, 0x7FF);
    std::uniform_int_distribution<int> dlc_dist(0, 8);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    std::cout << "Sending random CAN frames on " << ifname
              << " (Ctrl+C to stop)\n";
    std::cout << "Period: " << period_ms << " ms\n";
    if (use_fixed_id) std::cout << "Fixed ID: 0x" << std::hex << fixed_id << std::dec << "\n";

    while (!g_stop) {
        struct can_frame frame {};
        frame.can_id  = use_fixed_id ? fixed_id : id_dist(rng);
        frame.can_dlc = static_cast<__u8>(dlc_dist(rng));

        for (int i = 0; i < frame.can_dlc; ++i) {
            frame.data[i] = static_cast<__u8>(byte_dist(rng));
        }

        ssize_t n = ::write(sock, &frame, sizeof(frame));
        if (n != (ssize_t)sizeof(frame)) {
            perror("write(can_frame)");
            break;
        }

        // Pretty print: ID 3-hex, bytes 2-hex
        std::cout << "TX id=0x"
                  << std::hex << std::setw(3) << std::setfill('0')
                  << (frame.can_id & CAN_SFF_MASK)
                  << std::dec
                  << " dlc=" << static_cast<int>(frame.can_dlc)
                  << " data=";

        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::dec << "\n";

        if (period_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }
    }

    std::cout << "Stopping.\n";
    ::close(sock);
    return 0;
}
