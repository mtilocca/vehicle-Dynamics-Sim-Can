// src/can/socketcan_iface.cpp
#include "can/socketcan_iface.hpp"
#include "utils/logging.hpp"

#include <cstring>
#include <cerrno>

#include <fcntl.h>
#include <poll.h>
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
    if (sock_ < 0) {
        LOG_ERROR("socket() failed: %s", std::strerror(errno));
        return false;
    }

    struct ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        LOG_ERROR("ioctl(SIOCGIFINDEX) failed for %s: %s", 
                  ifname.c_str(), std::strerror(errno));
        close();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG_ERROR("bind() failed: %s", std::strerror(errno));
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
    if (n < 0) {
        LOG_ERROR("read() failed: %s", std::strerror(errno));
        return false;
    }
    return n == static_cast<ssize_t>(sizeof(out));
}

bool SocketCanIface::read_nonblocking(struct can_frame& out) {
    if (sock_ < 0) {
        return false;
    }
    
    // Save current socket flags
    int flags = ::fcntl(sock_, F_GETFL, 0);
    if (flags < 0) {
        LOG_ERROR("fcntl(F_GETFL) failed: %s", std::strerror(errno));
        return false;
    }
    
    // Set socket to non-blocking mode
    if (::fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
        LOG_ERROR("fcntl(F_SETFL, O_NONBLOCK) failed: %s", std::strerror(errno));
        return false;
    }
    
    // Attempt to read
    ssize_t nbytes = ::read(sock_, &out, sizeof(struct can_frame));
    
    // Restore original flags (blocking mode)
    ::fcntl(sock_, F_SETFL, flags);
    
    if (nbytes < 0) {
        // EAGAIN/EWOULDBLOCK means no data available (not an error)
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // No data available
        }
        
        // Other errors are real errors
        LOG_ERROR("read() failed: %s", std::strerror(errno));
        return false;
    }
    
    if (nbytes < (ssize_t)sizeof(struct can_frame)) {
        LOG_ERROR("Incomplete CAN frame read: %zd bytes (expected %zu)", 
                  nbytes, sizeof(struct can_frame));
        return false;
    }
    
    return true;  // Successfully read frame
}

bool SocketCanIface::read_timeout(struct can_frame& out, int timeout_ms) {
    if (sock_ < 0) {
        return false;
    }
    
    // Use poll() to wait for data with timeout
    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;
    
    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        LOG_ERROR("poll() failed: %s", std::strerror(errno));
        return false;
    }
    
    if (ret == 0) {
        // Timeout - no data available
        return false;
    }
    
    // Data available, read it
    if (pfd.revents & POLLIN) {
        ssize_t nbytes = ::read(sock_, &out, sizeof(struct can_frame));
        if (nbytes < 0) {
            LOG_ERROR("read() failed: %s", std::strerror(errno));
            return false;
        }
        return (nbytes == sizeof(struct can_frame));
    }
    
    return false;
}

bool SocketCanIface::write_frame(const struct can_frame& frame) {
    if (sock_ < 0) return false;
    const ssize_t n = ::write(sock_, &frame, sizeof(frame));
    if (n < 0) {
        LOG_ERROR("write() failed: %s", std::strerror(errno));
        return false;
    }
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

    if (::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER,
                     filters.data(),
                     static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0) {
        LOG_ERROR("setsockopt(CAN_RAW_FILTER) failed: %s", std::strerror(errno));
        return false;
    }
    
    return true;
}

} // namespace can