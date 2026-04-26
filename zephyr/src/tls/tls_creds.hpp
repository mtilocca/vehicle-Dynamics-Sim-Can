#pragma once
// zephyr/src/tls/tls_creds.hpp
// Common TLS credential store for HDV-Sim firmware.
//
// Usage:
//   Instantiate ZephyrTlsCredStore and call init() once before any TLS socket.
//   Use server_tag() for HTTPS server sockets; ca_tag() for MQTTS broker verify.

#include "tls/i_tls_cred_store.hpp"

// Sec-tag for the server certificate + private key pair.
#define HDV_TLS_SERVER_TAG  1

// Sec-tag for the external MQTT broker CA certificate.
#define HDV_TLS_CA_TAG      2

namespace tls {

class ZephyrTlsCredStore : public ITlsCredStore {
public:
    int init() override;
    int server_tag() const override { return HDV_TLS_SERVER_TAG; }
    int ca_tag()     const override { return HDV_TLS_CA_TAG; }
};

} // namespace tls

// C shim — kept for callers that haven't migrated to ZephyrTlsCredStore yet.
int tls_creds_init(void);
