// zephyr/src/utils/sock_guard.hpp
// RAII wrapper for zsock file descriptors.
// Calls zsock_close() on destruction unless release()d first.
//
// Usage:
//   hdv::SockGuard client(zsock_accept(...));
//   if (!client.valid()) { continue; }
//   // ... use client.get() ...
//   // closes automatically at end of scope
#pragma once

#include <zephyr/net/socket.h>

namespace hdv {

class SockGuard {
public:
    explicit SockGuard(int fd = -1) : fd_(fd) {}

    ~SockGuard() { close(); }

    // Move-only: transferring ownership is permitted.
    SockGuard(SockGuard&& o) noexcept : fd_(o.release()) {}
    SockGuard& operator=(SockGuard&&) = delete;
    SockGuard(const SockGuard&)       = delete;
    SockGuard& operator=(const SockGuard&) = delete;

    int  get()   const { return fd_; }
    bool valid() const { return fd_ >= 0; }

    // Relinquish ownership without closing.
    int release() { int f = fd_; fd_ = -1; return f; }

    // Close explicitly (safe to call multiple times).
    void close() {
        if (fd_ >= 0) {
            zsock_close(fd_);
            fd_ = -1;
        }
    }

private:
    int fd_;
};

} // namespace hdv
