#pragma once

#include <cstdint>
#include <cstring>

namespace sim {

// Command object updated from CAN RX (ACTUATOR_CMD_1 frame).
// Keep it POD-ish and deterministic.
struct ActuatorCmd {
    // From CAN:
    bool     system_enable = false;   // 0/1
    uint8_t  mode = 0;                // 0..7 (V1: unused but preserved)

    // Steering command in degrees (matches your CAN map scaling directly)
    double   steer_cmd_deg = 0.0;     // [-500..500] per your map, but you will clamp in SteerPlant

    // Drive torque command in Nm (matches CAN map)
    double   drive_torque_cmd_nm = 0.0; // [-4000..4000] per your map

    // Brake command in percent (0..100) (matches CAN map scaling)
    double   brake_cmd_pct = 0.0;

    // --- Quality-of-life ---
    // Timestamp (optional): sim time when last updated from RX.
    double   last_update_t_s = -1.0;

    // Reset to defaults
    void reset() { *this = ActuatorCmd{}; }

    // Consider the command "valid" if enabled (simple V1 policy)
    bool enabled() const { return system_enable; }
};

} // namespace sim
