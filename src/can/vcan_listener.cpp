#include <cstring>
#include <iostream>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

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

int main(int argc, char** argv) {
    const char* ifname = (argc > 1) ? argv[1] : "vcan0";
    int sock = open_can_socket(ifname);
    if (sock < 0) return 1;

    std::cout << "VCan Listener - Tool listening to virtual CAN messages on Channel VCAN0\n";
    std::cout << "Listening on " << ifname << " (Ctrl+C to stop)\n";

    while (true) {
        struct can_frame frame {};
        ssize_t n = ::read(sock, &frame, sizeof(frame));
        if (n < 0) {
            perror("read(can_frame)");
            ::close(sock);
            return 1;
        }
        if (n != (ssize_t)sizeof(frame)) continue;

        std::cout << "RX id=0x" << std::hex << (frame.can_id & CAN_SFF_MASK)
                  << std::dec << " dlc=" << (int)frame.can_dlc << " data=";
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << std::hex << (int)frame.data[i] << " ";
        }
        std::cout << std::dec << "\n";
    }

    ::close(sock);
    return 0;
}
