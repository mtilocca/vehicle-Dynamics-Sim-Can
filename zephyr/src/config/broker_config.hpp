// zephyr/src/config/broker_config.hpp
// BrokerConfig: MQTT broker connection parameters.
// Extracted from the scattered globals in mqtt_client.cpp.
//
// Extern declaration for consumers:
//   extern config::BrokerConfig g_broker_cfg;
#pragma once

#include <stdint.h>

namespace config {

struct BrokerConfig {
    char addr[32]      = "192.168.1.106";
    int  port          = 8883;
    bool reconnect_req = false;
};

} // namespace config
