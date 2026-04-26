#pragma once
// zephyr/src/stats/sys_stats.hpp
// Shared system telemetry struct — populated by stats_thread.cpp at 1 Hz,
// read by http_page.cpp for the dashboard "System Resources" card.

#include <stddef.h>
#include <stdint.h>

namespace hdv {

struct SysStats {
    uint32_t plant_loop_us_max   = 0;  // worst-case simulator loop since boot (μs)
    uint32_t can_rx_total        = 0;
    uint32_t can_timeout_total   = 0;
    size_t   heap_used           = 0;  // bytes allocated (0 if CONFIG_SYS_HEAP_RUNTIME_STATS=n)
    size_t   heap_free           = 0;
};

} // namespace hdv
