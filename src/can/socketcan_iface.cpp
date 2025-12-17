#include "can/socketcan_iface.hpp"

#include <cstring>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace can {

SocketCanIface::~SocketCanIface() {
    close();
}

bool SocketCanIface::open(const std::string& ifname) {
    close();

    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) return false;

    struct ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        close();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close();
        return false;
    }

    return true;
}

void SocketCanIface::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

bool SocketCanIface::read_frame(struct can_frame& out) {
    if (sock_ < 0) return false;
    const ssize_t n = ::read(sock_, &out, sizeof(out));
    return n == static_cast<ssize_t>(sizeof(out));
}

bool SocketCanIface::write_frame(const struct can_frame& frame) {
    if (sock_ < 0) return false;
    const ssize_t n = ::write(sock_, &frame, sizeof(frame));
    return n == static_cast<ssize_t>(sizeof(frame));
}

bool SocketCanIface::set_filters(const std::vector<uint32_t>& ids_11bit) {
    if (sock_ < 0) return false;

    if (ids_11bit.empty()) {
        // Clear filters => receive everything
        return ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) == 0;
    }

    std::vector<struct can_filter> filters;
    filters.reserve(ids_11bit.size());
    for (uint32_t id : ids_11bit) {
        struct can_filter f{};
        f.can_id   = id & CAN_SFF_MASK;
        f.can_mask = CAN_SFF_MASK;
        filters.push_back(f);
    }

    return ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER,
                        filters.data(),
                        static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) == 0;
}

} // namespace can