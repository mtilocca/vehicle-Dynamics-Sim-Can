// src/can/socketcan_iface.hpp
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

    // Blocking read/write
    bool read_frame(struct can_frame& out);
    bool write_frame(const struct can_frame& frame);
    
    /**
     * Read a CAN frame (non-blocking)
     * 
     * Returns true if frame received, false if no data available (EAGAIN/EWOULDBLOCK)
     * 
     * This is the KEY function for CAN RX closed-loop mode.
     * It allows polling the CAN socket without blocking the simulation loop.
     * 
     * Usage:
     *   struct can_frame rx_frame;
     *   while (can_iface.read_nonblocking(rx_frame)) {
     *       // Process frame
     *   }
     */
    bool read_nonblocking(struct can_frame& out);
    
    /**
     * Read a CAN frame (blocking with timeout)
     * 
     * @param out Output CAN frame
     * @param timeout_ms Timeout in milliseconds
     * @return true if frame received within timeout, false if timeout or error
     */
    bool read_timeout(struct can_frame& out, int timeout_ms);

    // Optional filter: only receive certain 11-bit IDs
    bool set_filters(const std::vector<uint32_t>& ids_11bit);

private:
    int sock_ = -1;
};

} // namespace can