// zephyr/src/shell/shell_network.cpp
// Shell: network subcommands (mac, IP).

#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>

#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static int cmd_network_mac(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    struct net_if* iface = net_if_get_default();
    if (!iface) {
        shell_error(sh, "No network interface found");
        return -ENODEV;
    }
    struct net_linkaddr* ll = net_if_get_link_addr(iface);
    shell_print(sh, "MAC : %02X:%02X:%02X:%02X:%02X:%02X",
                ll->addr[0], ll->addr[1], ll->addr[2],
                ll->addr[3], ll->addr[4], ll->addr[5]);
    shell_print(sh, "IP  : 192.168.1.80");
    shell_print(sh, "Port: 443  (https://192.168.1.80)");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(network_cmds,
    SHELL_CMD(mac, NULL, "Print MAC and IP address", cmd_network_mac),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(network, &network_cmds, "Network commands", NULL);
