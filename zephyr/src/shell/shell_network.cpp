// zephyr/src/shell/shell_network.cpp
// Shell: network subcommands — Wi-Fi status (SSID, RSSI, IP).

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/logging/log.h>

#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static int cmd_network_status(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;

    struct net_if* iface = net_if_get_default();
    if (!iface) {
        shell_error(sh, "No network interface found");
        return -ENODEV;
    }

    /* Wi-Fi connection info */
    struct wifi_iface_status ws{};
    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &ws, sizeof(ws)) == 0 &&
        ws.state == WIFI_STATE_COMPLETED) {
        shell_print(sh, "Wi-Fi: connected");
        shell_print(sh, "SSID : %.*s", ws.ssid_len, ws.ssid);
        shell_print(sh, "RSSI : %d dBm", ws.rssi);
        shell_print(sh, "Chan : %d", ws.channel);
    } else {
        shell_print(sh, "Wi-Fi: not connected (state=%d)", ws.state);
    }

    shell_print(sh, "IP   : 192.168.1.80");
    shell_print(sh, "Port : 443  (https://192.168.1.80)");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(network_cmds,
    SHELL_CMD(status, NULL, "Print Wi-Fi status and IP", cmd_network_status),
    /* keep 'mac' as alias for muscle-memory compatibility */
    SHELL_CMD(mac,    NULL, "Alias for 'network status'",  cmd_network_status),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(network, &network_cmds, "Network commands", NULL);
