#pragma once
// zephyr/src/can/zephyr_can_iface.hpp
// Drop-in replacement for SocketCanIface on Zephyr.
// Uses Zephyr's can_send() / can_add_rx_filter_msgq() API internally.
// The public interface uses our SocketCAN-compatible can_frame struct
// (from can_frame_compat.hpp) so no changes are needed in the rest of the code.

#include "can/can_frame_compat.hpp"
#include <cstdint>

// Forward-declare Zephyr types to avoid pulling <zephyr/drivers/can.h>
// into every translation unit.
struct device;
struct k_msgq;

namespace can {

class ZephyrCanIface {
public:
    ZephyrCanIface() = default;
    ~ZephyrCanIface();

    // Open the CAN device specified by Zephyr DeviceTree alias.
    // devname: DT alias (e.g. "fdcan1") or NULL to use zephyr,canbus chosen.
    bool open(const char* devname = nullptr);
    void close();

    // Write one frame to the bus (blocking, 100 ms timeout).
    bool write_frame(const struct can_frame& frame);

    // Non-blocking read from the software msgq (returns false if empty).
    bool read_nonblocking(struct can_frame& frame);

    // Register a hardware RX filter so matching frames are pushed into msgq.
    // Call once per RX frame ID after open().
    bool add_rx_filter_msgq(struct k_msgq* msgq, uint32_t can_id);

    bool is_open() const { return dev_ != nullptr; }

private:
    const struct device* dev_ = nullptr;
    int filter_id_ = -1;
};

} // namespace can
