#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <linux/can.h>

namespace can {

class SocketCanIface {
public:
    SocketCanIface() = default;
    ~SocketCanIface();

    SocketCanIface(const SocketCanIface&) = delete;
    SocketCanIface& operator=(const SocketCanIface&) = delete;

    bool open(const std::string& ifname);
    void close();

    bool is_open() const { return sock_ >= 0; }

    // Blocking read/write (fine for now)
    bool read_frame(struct can_frame& out);
    bool write_frame(const struct can_frame& frame);

    // Optional filter: only receive certain 11-bit IDs
    bool set_filters(const std::vector<uint32_t>& ids_11bit);

private:
    int sock_ = -1;
};

} // namespace can