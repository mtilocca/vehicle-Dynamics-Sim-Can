// zephyr/src/shell/shell_can.cpp
// Shell: CAN stats, rx_frame, map, tx_test, timeout subcommands.

#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>

#include "sim/actuator_cmd.hpp"
#include "can/can_map_static.hpp"
#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// Runtime-adjustable CAN RX watchdog timeout (seconds).
// plant_thread.cpp reads this via shell_cmds::g_can_rx_timeout_s.
namespace shell_cmds {
double g_can_rx_timeout_s = 0.5;
} // namespace shell_cmds

extern "C" int can_tx_test_frame(double steer_deg, double torque_nm,
                                  double brake_pct, bool enable);

static int cmd_can_stats(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;

    double last_rx;
    { hdv::MutexGuard g(g_sim_cmd_mtx); last_rx = g_sim_bus.cmd.last_update_t_s; }

    shell_print(sh, "--- CAN Stats ---");
    shell_print(sh, "  TX frames  : %u", (uint32_t)atomic_get(&g_ctrl_bus.can_tx_count));
    shell_print(sh, "  RX frames  : %u", (uint32_t)atomic_get(&g_ctrl_bus.can_rx_count));
    shell_print(sh, "  RX timeouts: %u", (uint32_t)atomic_get(&g_ctrl_bus.can_timeout_count));
    shell_print(sh, "  Last RX    : %.3f s", last_rx);
    shell_print(sh, "  RX timeout : %.3f s", shell_cmds::g_can_rx_timeout_s);
    return 0;
}

static int cmd_can_rx_frame(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    sim::ActuatorCmd c;
    { hdv::MutexGuard g(g_sim_cmd_mtx); c = g_sim_bus.cmd; }

    shell_print(sh, "--- Last ACTUATOR_CMD_1 ---");
    shell_print(sh, "  system_enable   = %d",   (int)c.system_enable);
    shell_print(sh, "  gear_position   = %d",   (int)c.gear_position);
    shell_print(sh, "  drive_torque_nm = %.1f", c.drive_torque_cmd_nm);
    shell_print(sh, "  brake_pct       = %.2f", c.brake_cmd_pct);
    shell_print(sh, "  steer_deg       = %.2f", c.steer_cmd_deg);
    shell_print(sh, "  last_update_t_s = %.3f", c.last_update_t_s);
    return 0;
}

static int cmd_can_tx_test(const struct shell* sh, size_t argc, char** argv)
{
    if (!shell_cmds::require_login(sh)) return -EACCES;

    double steer  = (argc > 1) ? strtod(argv[1], nullptr) : 10.0;
    double torque = (argc > 2) ? strtod(argv[2], nullptr) : 50000.0;
    double brake  = (argc > 3) ? strtod(argv[3], nullptr) : 0.0;

    shell_print(sh, "Sending ACTUATOR_CMD_1: steer=%.1f  torque=%.0f Nm  brake=%.1f %%",
                steer, torque, brake);
    int ret = can_tx_test_frame(steer, torque, brake, true);
    if (ret < 0) {
        shell_error(sh, "can_send failed: %d", ret);
        return ret;
    }
    shell_print(sh, "Sent OK.");
    return 0;
}

static int cmd_can_map(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Static CAN Map (%u frames, %u RX, %u TX) ---",
                (unsigned)can_static::k_frame_count,
                (unsigned)can_static::k_rx_count,
                (unsigned)can_static::k_tx_count);
    for (uint8_t i = 0; i < can_static::k_frame_count; ++i) {
        const can_static::FrameDef& f = can_static::k_frames[i];
        shell_print(sh, "  [%s] 0x%08X  %-22s  %2u sig(s)  %3u ms",
                    f.is_rx ? "RX" : "TX",
                    (unsigned)f.id, f.name,
                    (unsigned)f.sig_count, (unsigned)f.cycle_ms);
    }
    return 0;
}

static int cmd_can_timeout(const struct shell* sh, size_t argc, char** argv)
{
    if (argc < 2) {
        shell_print(sh, "Current CAN RX watchdog timeout: %.0f ms",
                    shell_cmds::g_can_rx_timeout_s * 1000.0);
        return 0;
    }
    double ms = strtod(argv[1], nullptr);
    if (ms < 100.0 || ms > 5000.0) {
        shell_error(sh, "Timeout must be 100..5000 ms");
        return -EINVAL;
    }
    shell_cmds::g_can_rx_timeout_s = ms / 1000.0;
    shell_print(sh, "CAN RX watchdog timeout set to %.0f ms", ms);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(can_cmds,
    SHELL_CMD(stats,    NULL, "CAN TX/RX counters",                  cmd_can_stats),
    SHELL_CMD(rx_frame, NULL, "Last decoded actuator cmd",            cmd_can_rx_frame),
    SHELL_CMD(map,      NULL, "Dump static CAN frame map",            cmd_can_map),
    SHELL_CMD(tx_test,  NULL, "Send test frame (requires login)",     cmd_can_tx_test),
    SHELL_CMD(timeout,  NULL, "Get/set CAN RX watchdog timeout (ms)", cmd_can_timeout),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can, &can_cmds, "CAN bus commands", NULL);
