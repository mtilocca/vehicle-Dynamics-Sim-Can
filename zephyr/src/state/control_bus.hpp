// zephyr/src/state/control_bus.hpp
// ControlBus: source arbitration + all atomic counters + surface friction.
// All fields are atomics or plain double (benign races acceptable for mu).
// No mutex required.
//
// CtrlSource enum lives here (moved from mqtt_client.hpp) so that can/,
// http/, and shell/ modules can include this header without pulling in
// the MQTT implementation headers.
//
// Extern declaration for consumers:
//   extern hdv::ControlBus g_ctrl_bus;
#pragma once

#include <zephyr/sys/atomic.h>

namespace hdv {

enum class CtrlSource : int {
    CAN  = 0,
    MQTT = 1,
    HTTP = 2,
};

// Integer shims for use with atomic_set/atomic_get (which return long).
static constexpr int CTRL_CAN  = static_cast<int>(CtrlSource::CAN);
static constexpr int CTRL_MQTT = static_cast<int>(CtrlSource::MQTT);
static constexpr int CTRL_HTTP = static_cast<int>(CtrlSource::HTTP);

struct ControlBus {
    atomic_t ctrl_source       = ATOMIC_INIT(static_cast<int>(CtrlSource::CAN));
    atomic_t mqtt_rx_count     = ATOMIC_INIT(0);
    atomic_t can_tx_count      = ATOMIC_INIT(0);
    atomic_t can_rx_count      = ATOMIC_INIT(0);
    atomic_t can_timeout_count = ATOMIC_INIT(0);
    double   surface_mu        = 0.72;  // shell write / plant read — benign
};

} // namespace hdv
