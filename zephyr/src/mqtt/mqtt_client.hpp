// zephyr/src/mqtt/mqtt_client.hpp
// MQTT client public API — subscribe to hdv/cmd/actuator, publish hdv/state/vehicle.

#pragma once

#include <stddef.h>
#include "state/control_bus.hpp"

// Re-export for files that include only mqtt_client.hpp (not control_bus.hpp directly).
using hdv::CTRL_CAN;
using hdv::CTRL_MQTT;
using hdv::CTRL_HTTP;

// Broker address and port — defined in mqtt_client.cpp, written only by the
// HTTP thread (apply_web_cmd) or mqtt_request_reconnect().
// The MQTT thread reads these only at connect time, so a plain write suffices.
extern char g_mqtt_broker_addr[32];
extern int  g_mqtt_broker_port;
extern bool g_mqtt_reconnect_req;   // set true to trigger a reconnect from outside

// Whether the MQTT thread is currently connected (read-only for dashboard/shell).
extern bool g_mqtt_connected;
