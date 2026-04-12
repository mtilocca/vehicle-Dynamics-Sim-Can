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

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// Defined in main.cpp
extern SysStats       g_sys_stats;
extern struct k_mutex g_stats_mutex;
extern atomic_t       g_can_rx_count;
extern atomic_t       g_can_timeout_count;

static void stats_thread(void*, void*, void*)
{
    while (true) {
        k_msleep(1000);

        struct sys_memory_stats heap_stats{};
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
        extern struct sys_heap _system_heap;
        sys_heap_runtime_stats_get(&_system_heap, &heap_stats);
#endif

        k_mutex_lock(&g_stats_mutex, K_FOREVER);
        g_sys_stats.can_rx_total      = (uint32_t)atomic_get(&g_can_rx_count);
        g_sys_stats.can_timeout_total = (uint32_t)atomic_get(&g_can_timeout_count);
        g_sys_stats.heap_used         = heap_stats.allocated_bytes;
        g_sys_stats.heap_free         = heap_stats.free_bytes;
        // plant_loop_us_max is written by plant_thread under g_stats_mutex
        k_mutex_unlock(&g_stats_mutex);
    }
}

K_THREAD_DEFINE(stats_tid, 1024, stats_thread, NULL, NULL, NULL, 11, 0, 0);
