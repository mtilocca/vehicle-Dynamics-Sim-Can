// zephyr/src/shell/shell_stats.cpp
// Shell: stats, mem, threads commands.

#include <zephyr/logging/log.h>

#include "stats/sys_stats.hpp"
#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static int cmd_stats(const struct shell* sh, size_t argc, char** argv)
{
    if (argc > 1 && strncmp(argv[1], "reset", 5) == 0) {
        { hdv::MutexGuard g(g_health_mtx); g_health_bus.stats.plant_loop_us_max = 0; }
        shell_print(sh, "Stats counters reset");
        return 0;
    }

    SysStats s;
    { hdv::MutexGuard g(g_health_mtx); s = g_health_bus.stats; }

    shell_print(sh, "--- System Stats ---");
    shell_print(sh, "  Plant loop max : %u us", s.plant_loop_us_max);
    shell_print(sh, "  CAN RX total   : %u",    s.can_rx_total);
    shell_print(sh, "  CAN timeouts   : %u",    s.can_timeout_total);
    shell_print(sh, "  Heap used      : %zu B",  s.heap_used);
    shell_print(sh, "  Heap free      : %zu B",  s.heap_free);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(stats_sub,
    SHELL_CMD(reset, NULL, "Reset stats counters", cmd_stats),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(stats, &stats_sub, "System health stats [reset]", cmd_stats);

static int cmd_mem(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    SysStats s;
    { hdv::MutexGuard g(g_health_mtx); s = g_health_bus.stats; }

    shell_print(sh, "--- Heap Memory ---");
    shell_print(sh, "  Used : %zu B", s.heap_used);
    shell_print(sh, "  Free : %zu B", s.heap_free);
    if (s.heap_used + s.heap_free > 0) {
        uint32_t pct = (uint32_t)(s.heap_used * 100 /
                                  (s.heap_used + s.heap_free));
        shell_print(sh, "  Usage: %u %%", pct);
    }
    return 0;
}
SHELL_CMD_REGISTER(mem, NULL, "Heap memory usage", cmd_mem);

static void print_thread_cb(const struct k_thread* t, void* user_data)
{
    const struct shell* sh = (const struct shell*)user_data;
    const char* name = k_thread_name_get((struct k_thread*)t);
    if (!name || name[0] == '\0') name = "(unnamed)";

    size_t unused = 0;
#if defined(CONFIG_THREAD_STACK_INFO)
    k_thread_stack_space_get(t, &unused);
#endif

    shell_print(sh, "  %-16s prio=%-3d  stack_free=%zu B  state=0x%x",
                name, t->base.prio, unused, t->base.thread_state);
}

static int cmd_threads(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Threads ---");
    k_thread_foreach(print_thread_cb, (void*)sh);
    return 0;
}
SHELL_CMD_REGISTER(threads, NULL, "List all threads with stack info", cmd_threads);
