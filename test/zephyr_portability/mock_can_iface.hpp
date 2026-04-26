#pragma once
// Mock ICanIface for host-side unit tests.
#include "can/i_can_iface.hpp"
#include "can/can_frame_compat.hpp"
#include <vector>

namespace can {

class MockCanIface : public ICanIface {
public:
    bool open(const char* = nullptr) override { open_called = true; return open_result; }
    void close() override { close_called = true; }
    bool write_frame(const ::can_frame& f) override { tx_frames.push_back(f); return true; }
    bool read_nonblocking(::can_frame& f) override {
        if (rx_frames.empty()) return false;
        f = rx_frames.front();
        rx_frames.erase(rx_frames.begin());
        return true;
    }
    bool add_rx_filter_msgq(struct k_msgq*, uint32_t) override { return true; }
    bool is_open() const override { return open_called && !close_called; }

    // Test helpers
    std::vector<::can_frame> tx_frames;
    std::vector<::can_frame> rx_frames;
    bool open_called  = false;
    bool close_called = false;
    bool open_result  = true;
};

} // namespace can
