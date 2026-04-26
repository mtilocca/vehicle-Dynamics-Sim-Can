// zephyr/src/mqtt/i_mqtt_client.hpp
// IMqttClient: pure-virtual MQTT client interface.
// ZephyrMqttClient implements this; host-side mock also implements it.
#pragma once

#include <stddef.h>

namespace mqtt {

class IMqttClient {
public:
    virtual ~IMqttClient() = default;

    // Connect to the configured broker. Returns 0 on success, negative errno on failure.
    virtual int  connect()    = 0;
    virtual void disconnect() = 0;

    virtual bool is_connected() const = 0;

    // Publish payload to topic. Returns 0 on success.
    virtual int publish(const char* topic, const void* payload, size_t len) = 0;

    // Process pending MQTT I/O (call from polling loop). Returns 0 or negative errno.
    virtual int poll() = 0;
};

} // namespace mqtt
