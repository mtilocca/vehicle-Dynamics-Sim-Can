// zephyr/src/tls/i_tls_cred_store.hpp
// ITlsCredStore: pure-virtual TLS credential store interface.
// ZephyrTlsCredStore implements this; host-side stub also implements it.
#pragma once

namespace tls {

class ITlsCredStore {
public:
    virtual ~ITlsCredStore() = default;

    // Load credentials into the platform credential store.
    // Returns 0 on success, negative errno on failure (e.g. -EEXIST if already loaded).
    virtual int init() = 0;

    // Sec-tag to use when configuring TLS sockets (server cert + key pair).
    virtual int server_tag() const = 0;

    // Sec-tag reserved for external broker CA certificate.
    virtual int ca_tag() const = 0;
};

} // namespace tls
