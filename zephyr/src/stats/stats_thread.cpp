// zephyr/src/stats/stats_thread.cpp
// System telemetry thread — runs at 1 Hz, samples metrics and stores
// them in g_sys_stats for shell 'stats' command and HTTP dashboard.
//
// Priority 11 — below HTTP (10), above LED (12).
// Stack 1024 B.

#include <zephyr/kernel.h>
#include <zephyr/sys/mem_stats.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "sys_stats.hpp"
#include "utils/mutex_guard.hpp"
#include "state/control_bus.hpp"
#include "state/system_health.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// Defined in main.cpp
extern hdv::ControlBus       g_ctrl_bus;
extern hdv::SystemHealthBus  g_health_bus;
extern struct k_mutex        g_health_mtx;

static void stats_thread(void*, void*, void*)
{
    while (true) {
        k_msleep(1000);

        struct sys_memory_stats heap_stats{};
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
        extern struct sys_heap _system_heap;
        sys_heap_runtime_stats_get(&_system_heap, &heap_stats);
#endif

        {
            hdv::MutexGuard g(g_health_mtx);
            g_health_bus.stats.can_rx_total      = (uint32_t)atomic_get(&g_ctrl_bus.can_rx_count);
            g_health_bus.stats.can_timeout_total = (uint32_t)atomic_get(&g_ctrl_bus.can_timeout_count);
            g_health_bus.stats.heap_used         = heap_stats.allocated_bytes;
            g_health_bus.stats.heap_free         = heap_stats.free_bytes;
            // plant_loop_us_max is written by plant_thread under g_health_mtx
        }
    }
}

K_THREAD_DEFINE(stats_tid, 1024, stats_thread, NULL, NULL, NULL, 11, 0, 0);
