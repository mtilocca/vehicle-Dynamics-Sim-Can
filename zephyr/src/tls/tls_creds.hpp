#pragma once
// zephyr/src/tls/tls_creds.hpp
// Common TLS credential store for HDV-Sim firmware.
//
// Usage:
//   Call tls_creds_init() once before creating any TLS socket.
//   Use HDV_TLS_SERVER_TAG as the sec_tag when setting TLS_SEC_TAG_LIST on
//   HTTPS server sockets, or as the peer verify tag for MQTTS client sockets
//   connecting to a broker that presents the same certificate.
//   Use HDV_TLS_CA_TAG (reserved) when the MQTTS broker has its own CA cert.

// Sec-tag for the server certificate + private key pair.
// Registered by tls_creds_init() — reused by both the HTTPS server and
// any future MQTTS client that needs to identify or verify this device.
#define HDV_TLS_SERVER_TAG  1

// Reserved sec-tag for an external MQTT broker's CA certificate.
// Not registered until the MQTTS module is implemented.
#define HDV_TLS_CA_TAG      2

// Load server.crt and server.key (embedded at build time as .inc files) into
// the Zephyr TLS credential store under HDV_TLS_SERVER_TAG.
// Safe to call multiple times — returns -EEXIST on repeated calls.
// Returns 0 on success, negative errno on failure.
int tls_creds_init(void);
