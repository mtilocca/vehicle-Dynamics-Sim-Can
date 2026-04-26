// zephyr/src/shell/shell_mqtt.cpp
// Shell: MQTT status and reconnect subcommands.

#include <zephyr/logging/log.h>

#include "mqtt/mqtt_client.hpp"
#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static int cmd_mqtt_status(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    const char* src_str =
        (atomic_get(&g_ctrl_bus.ctrl_source) == CTRL_MQTT) ? "MQTT" :
        (atomic_get(&g_ctrl_bus.ctrl_source) == CTRL_HTTP) ? "HTTP" : "CAN";
    shell_print(sh, "Broker     : %s:%d", g_mqtt_broker_addr, g_mqtt_broker_port);
    shell_print(sh, "Connected  : %s", g_mqtt_connected ? "yes" : "no");
    shell_print(sh, "MQTT RX    : %u", (uint32_t)atomic_get(&g_ctrl_bus.mqtt_rx_count));
    shell_print(sh, "Ctrl source: %s", src_str);
    return 0;
}

static int cmd_mqtt_reconnect(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    g_mqtt_reconnect_req = true;
    shell_print(sh, "MQTT: reconnect requested → %s:%d",
                g_mqtt_broker_addr, g_mqtt_broker_port);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mqtt_cmds,
    SHELL_CMD(status,    NULL, "MQTT broker status and RX count", cmd_mqtt_status),
    SHELL_CMD(reconnect, NULL, "Force MQTT reconnect",            cmd_mqtt_reconnect),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(mqtt, &mqtt_cmds, "MQTT client commands", NULL);
