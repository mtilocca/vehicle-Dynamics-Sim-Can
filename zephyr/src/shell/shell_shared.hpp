#pragma once
// Internal header: shared externs and helpers for shell command files.
// Not a public API — include only from shell/*.cpp.

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#include "state/sim_state.hpp"
#include "state/control_bus.hpp"
#include "state/system_health.hpp"
#include "utils/mutex_guard.hpp"

// Shared state buses (defined in main.cpp)
extern hdv::SimStateBus      g_sim_bus;
extern hdv::ControlBus       g_ctrl_bus;
extern hdv::SystemHealthBus  g_health_bus;
extern struct k_mutex        g_sim_plant_mtx;
extern struct k_mutex        g_sim_cmd_mtx;
extern struct k_mutex        g_health_mtx;

namespace shell_cmds {

// Returns true if the shell is unlocked; prints an error and returns false otherwise.
bool require_login(const struct shell* sh);

// Runtime-adjustable CAN RX watchdog timeout (seconds).
// Defined in shell_can.cpp; read by plant_thread.cpp.
extern double g_can_rx_timeout_s;

} // namespace shell_cmds
