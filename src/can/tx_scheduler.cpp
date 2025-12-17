#include "can/tx_scheduler.hpp"

namespace can {

void TxScheduler::init(const std::vector<FrameDef>& frames) {
    frames_ = &frames;
    next_.assign(frames.size(), Clock::now());
}

void TxScheduler::force_all_due() {
    if (!frames_) return;
    auto now = Clock::now();
    for (auto& t : next_) t = now;
}

std::vector<size_t> TxScheduler::due(Clock::time_point now) {
    std::vector<size_t> out;
    if (!frames_) return out;

    for (size_t i = 0; i < frames_->size(); ++i) {
        const auto& f = (*frames_)[i];
        if (now >= next_[i]) {
            out.push_back(i);
            next_[i] = now + std::chrono::milliseconds(f.cycle_ms);
        }
    }
    return out;
}

} // namespace can