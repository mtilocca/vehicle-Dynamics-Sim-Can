// zephyr/src/mqtt/mqtt_client.hpp
// MQTT client public API — subscribe to hdv/cmd/actuator, publish hdv/state/vehicle.

#pragma once

#include <stddef.h>
#include "state/control_bus.hpp"
#include "config/broker_config.hpp"

// Re-export for files that include only mqtt_client.hpp (not control_bus.hpp directly).
using hdv::CTRL_CAN;
using hdv::CTRL_MQTT;
using hdv::CTRL_HTTP;

namespace mqtt {
// Returns a reference to the live broker config struct.
// Write addr/port then set reconnect_req=true to trigger a reconnect.
config::BrokerConfig& broker_config();
} // namespace mqtt

// Whether the MQTT thread is currently connected (read-only for dashboard/shell).
extern bool g_mqtt_connected;
