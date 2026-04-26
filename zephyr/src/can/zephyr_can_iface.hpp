#pragma once
// zephyr/src/can/zephyr_can_iface.hpp
// Zephyr-specific implementation of ICanIface.
// Uses Zephyr's can_send() / can_add_rx_filter_msgq() API internally.
// The public interface uses our SocketCAN-compatible can_frame struct
// (from can_frame_compat.hpp) so no changes are needed in the rest of the code.

#include "can/can_frame_compat.hpp"
#include "can/i_can_iface.hpp"
#include <cstdint>

// Forward-declare Zephyr types to avoid pulling <zephyr/drivers/can.h>
// into every translation unit.
struct device;
struct k_msgq;

namespace can {

class ZephyrCanIface : public ICanIface {
public:
    ZephyrCanIface() = default;
    ~ZephyrCanIface() override;

    bool open(const char* devname = nullptr) override;
    void close() override;
    bool write_frame(const struct can_frame& frame) override;
    bool read_nonblocking(struct can_frame& frame) override;
    bool add_rx_filter_msgq(struct k_msgq* msgq, uint32_t can_id) override;
    bool is_open() const override { return dev_ != nullptr; }

private:
    const struct device* dev_ = nullptr;
    int filter_id_ = -1;
};

} // namespace can
