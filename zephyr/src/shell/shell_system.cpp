// zephyr/src/shell/shell_system.cpp
// Shell: system subcommands (uptime).

#include <zephyr/logging/log.h>

#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static int cmd_system_uptime(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    uint32_t ms  = k_uptime_get_32();
    uint32_t s   = ms / 1000;
    uint32_t h   = s / 3600;
    uint32_t m   = (s % 3600) / 60;
    uint32_t sec = s % 60;
    shell_print(sh, "Uptime: %02u:%02u:%02u  (%u ms)", h, m, sec, ms);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(system_cmds,
    SHELL_CMD(uptime, NULL, "Print system uptime", cmd_system_uptime),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(system, &system_cmds, "System commands", NULL);
