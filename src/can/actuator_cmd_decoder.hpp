// src/can/actuator_cmd_decoder.hpp
#pragma once

#include "can/can_map.hpp"
#include "can/can_codec.hpp"
#include "sim/actuator_cmd.hpp"
#include <linux/can.h>
#include <string>

namespace can {

/**
 * Decodes ACTUATOR_CMD_1 CAN frames into ActuatorCmd structure
 * 
 * Matches encoding from Go controller (dds-fusion-core/utils/can_loader.go)
 * 
 * Frame 0x100 (ACTUATOR_CMD_1):
 *   system_enable       bit 0      (1 bit)
 *   mode                bit 1-3    (3 bits)
 *   steer_cmd_deg       bit 8-23   (16 bits signed, factor 0.1)
 *   drive_torque_cmd_nm bit 24-39  (16 bits signed, factor 1.0)
 *   brake_cmd_pct       bit 40-47  (8 bits unsigned, factor 0.5)
 */
class ActuatorCmdDecoder {
public:
    /**
     * Initialize decoder with CAN map and frame name
     * 
     * @param can_map Loaded CAN map with RX frame definitions
     * @param frame_name Name of actuator command frame (default: "ACTUATOR_CMD_1")
     * @throws std::runtime_error if frame not found in CAN map
     */
    explicit ActuatorCmdDecoder(const CanMap& can_map, 
                               const std::string& frame_name = "ACTUATOR_CMD_1");
    
    /**
     * Decode a CAN frame into ActuatorCmd
     * 
     * @param frame CAN frame received from socket
     * @param out_cmd Output ActuatorCmd structure (modified only if frame ID matches)
     * @param sim_time Current simulation time (stored in out_cmd.last_update_t_s)
     * @return true if frame ID matches and decode succeeded, false otherwise
     */
    bool decode(const struct can_frame& frame, 
                sim::ActuatorCmd& out_cmd, 
                double sim_time);
    
    /**
     * Get the expected CAN ID for this frame
     */
    uint32_t get_frame_id() const { return frame_id_; }
    
    /**
     * Get the frame name
     */
    const std::string& get_frame_name() const { return frame_name_; }
    
private:
    std::string frame_name_;
    uint32_t frame_id_;
    FrameDef frame_def_;
};

} // namespace can