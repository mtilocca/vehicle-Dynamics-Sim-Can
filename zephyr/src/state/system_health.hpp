// zephyr/src/state/system_health.hpp
// SystemHealthBus: runtime metrics written by stats_thread, read by HTTP + shell.
// Groups g_sys_stats into one named struct — eliminates the SysStats
// re-declaration that existed in plant_thread.cpp and debug_cmds.cpp.
//
// The mutex is declared separately (K_MUTEX_DEFINE in main.cpp).
//
// Extern declarations for consumers:
//   extern hdv::SystemHealthBus g_health_bus;
//   extern struct k_mutex       g_health_mtx;
#pragma once

#include "stats/sys_stats.hpp"

namespace hdv {

struct SystemHealthBus {
    SysStats stats{};
};

} // namespace hdv
