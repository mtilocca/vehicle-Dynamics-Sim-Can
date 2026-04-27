#pragma once
// net_mgr.hpp — Wi-Fi connection manager.
// Starts a Wi-Fi connect request and exposes a blocking wait for L4 ready.
// Call net_mgr_init() from main() after boot_write_img_confirmed().
// Threads that need the network (HTTPS, MQTT) call net_mgr_wait_connected()
// instead of sleeping a fixed time.

#include <zephyr/kernel.h>

void net_mgr_init();

// Block until NET_EVENT_L4_CONNECTED fires or timeout expires.
// Returns 0 on success, -EAGAIN on timeout.
int net_mgr_wait_connected(k_timeout_t timeout);
