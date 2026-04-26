// zephyr/src/utils/mutex_guard.hpp
// RAII scoped lock for Zephyr k_mutex.
// Acquires on construction (K_FOREVER), releases in destructor.
// Non-copyable, non-movable — always stack-allocated.
//
// Usage:
//   { hdv::MutexGuard g(g_state_mutex); local = g_state; }
#pragma once

#include <zephyr/kernel.h>

namespace hdv {

class MutexGuard {
public:
    explicit MutexGuard(struct k_mutex& m) : m_(m) {
        k_mutex_lock(&m_, K_FOREVER);
    }
    ~MutexGuard() { k_mutex_unlock(&m_); }

    MutexGuard(const MutexGuard&)            = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;
    MutexGuard(MutexGuard&&)                 = delete;
    MutexGuard& operator=(MutexGuard&&)      = delete;

private:
    struct k_mutex& m_;
};

} // namespace hdv
