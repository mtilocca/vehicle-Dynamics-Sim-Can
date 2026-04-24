// zephyr/src/mqtt/mqtt_client.hpp
// MQTT client public API — subscribe to hdv/cmd/actuator, publish hdv/state/vehicle.

#pragma once

#include <stddef.h>

// Control source arbitration — which source is allowed to write g_cmd.
// Stored in g_ctrl_source (atomic_t, defined in main.cpp).
enum CtrlSource : int {
    CTRL_CAN  = 0,
    CTRL_MQTT = 1,
    CTRL_HTTP = 2,
};

// Broker address and port — defined in mqtt_client.cpp, written only by the
// HTTP thread (apply_web_cmd) or mqtt_request_reconnect().
// The MQTT thread reads these only at connect time, so a plain write suffices.
extern char g_mqtt_broker_addr[32];
extern int  g_mqtt_broker_port;
extern bool g_mqtt_reconnect_req;   // set true to trigger a reconnect from outside

// Whether the MQTT thread is currently connected (read-only for dashboard/shell).
extern bool g_mqtt_connected;

// Total MQTT actuator commands received (mirrors g_can_rx_count for stats).
// Defined in main.cpp as atomic_t g_mqtt_rx_count.
