#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include "can/can_map.hpp"

namespace can {

// Simple scheduler: tells you which TX frames are due "now"
class TxScheduler {
public:
    using Clock = std::chrono::steady_clock;

    TxScheduler() = default;

    // Call once after loading map.tx_frames()
    void init(const std::vector<FrameDef>& frames);

    // Returns indices (into frames passed to init) that are due to send at time 'now'
    std::vector<size_t> due(Clock::time_point now);

    // Optionally: force all frames due immediately (useful on startup)
    void force_all_due();

private:
    const std::vector<FrameDef>* frames_ = nullptr;
    std::vector<Clock::time_point> next_;
};

} // namespace can