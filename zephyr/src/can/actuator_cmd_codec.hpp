#pragma once
// ACTUATOR_CMD_1 decode — no Zephyr deps, host-buildable for unit tests.
// Bit layout (Intel/LSB-first):
//   Byte 0  [0:0]   system_enable       unsigned, factor 1.0
//   Byte 0  [1:2]   gear_position       unsigned, factor 1.0
//   Byte 0  [3:4]   mode                unsigned, factor 1.0
//   Byte 1-2[8:23]  steer_cmd_deg       signed,   factor 0.1
//   Byte 3-4[24:39] drive_torque_cmd_nm signed,   factor 10.0
//   Byte 5  [40:47] brake_cmd_pct       unsigned, factor 1.0
//   Byte 6-7[48:63] seq_num             anti-replay counter (ignored here)

#include <cstdint>
#include "sim/actuator_cmd.hpp"

namespace can_codec {

inline double clamp_d(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Core decode: takes a raw 8-byte CAN payload pointer.
// Works with any can_frame type (Zephyr native or SocketCAN compat) via .data.
inline void decode_actuator_cmd(const uint8_t* d, sim::ActuatorCmd& c)
{
    c.system_enable = (d[0] >> 0) & 0x01u;
    c.mode          = (d[0] >> 3) & 0x03u;

    uint8_t gear_raw = (d[0] >> 1) & 0x03u;
    if (gear_raw < static_cast<uint8_t>(sim::GearPosition::RESERVED)) {
        c.gear_position = static_cast<sim::GearPosition>(gear_raw);
    } else {
        c.gear_position = sim::GearPosition::NEUTRAL;
    }

    int16_t steer_raw  = static_cast<int16_t>((uint16_t)d[1] | ((uint16_t)d[2] << 8));
    int16_t torque_raw = static_cast<int16_t>((uint16_t)d[3] | ((uint16_t)d[4] << 8));

    c.steer_cmd_deg       = clamp_d(steer_raw  * 0.1,     -45.0,     45.0);
    c.drive_torque_cmd_nm = clamp_d(torque_raw * 10.0,      0.0, 145000.0);
    c.brake_cmd_pct       = clamp_d((double)d[5],           0.0,    100.0);
}

} // namespace can_codec
