// zephyr/src/can/i_can_iface.hpp
// ICanIface: pure-virtual CAN bus interface.
// ZephyrCanIface implements this; host-side mock also implements it.
#pragma once

#include "can/can_frame_compat.hpp"
#include <cstdint>

struct k_msgq;

namespace can {

class ICanIface {
public:
    virtual ~ICanIface() = default;

    virtual bool open(const char* devname = nullptr) = 0;
    virtual void close()                             = 0;

    virtual bool write_frame(const struct can_frame& frame)            = 0;
    virtual bool read_nonblocking(struct can_frame& frame)             = 0;
    virtual bool add_rx_filter_msgq(struct k_msgq* msgq, uint32_t id) = 0;

    virtual bool is_open() const = 0;
};

} // namespace can
