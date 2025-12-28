// src/can/actuator_cmd_decoder.cpp
#include "can/actuator_cmd_decoder.hpp"
#include "utils/logging.hpp"
#include <stdexcept>

namespace can {

ActuatorCmdDecoder::ActuatorCmdDecoder(const CanMap& can_map, const std::string& frame_name)
    : frame_name_(frame_name)
    , frame_id_(0)
{
    // Find frame definition in RX frames
    // rx_frames() returns std::unordered_map<uint32_t, FrameDef>
    bool found = false;
    for (const auto& kv : can_map.rx_frames()) {
        const FrameDef& fd = kv.second;  // Extract FrameDef from pair
        if (fd.frame_name == frame_name) {
            frame_def_ = fd;
            frame_id_ = fd.frame_id;
            found = true;
            break;
        }
    }
    
    if (!found) {
        LOG_ERROR("Actuator command frame '%s' not found in CAN map RX frames", 
                  frame_name.c_str());
        throw std::runtime_error("Frame not found: " + frame_name);
    }
    
    // Validate frame has expected signals
    const char* required_signals[] = {
        "system_enable",
        "mode",
        "steer_cmd_deg",
        "drive_torque_cmd_nm",
        "brake_cmd_pct"
    };
    
    for (const char* sig_name : required_signals) {
        bool has_signal = false;
        for (const auto& sig : frame_def_.signals) {
            if (sig.signal_name == sig_name) {  // Use signal_name, not name
                has_signal = true;
                break;
            }
        }
        if (!has_signal) {
            LOG_WARN("Frame %s missing expected signal: %s", 
                     frame_name.c_str(), sig_name);
        }
    }
    
    LOG_INFO("ActuatorCmdDecoder initialized: frame=%s id=0x%03X dlc=%d signals=%zu", 
             frame_name.c_str(), frame_id_, frame_def_.dlc, frame_def_.signals.size());
}

bool ActuatorCmdDecoder::decode(const struct can_frame& frame, 
                                sim::ActuatorCmd& out_cmd, 
                                double sim_time) {
    // Verify frame ID matches
    if (frame.can_id != frame_id_) {
        return false;
    }
    
    // Verify DLC matches
    if (frame.can_dlc != frame_def_.dlc) {
        LOG_WARN("CAN RX frame 0x%03X has unexpected DLC: %d (expected %d)",
                 frame.can_id, frame.can_dlc, frame_def_.dlc);
        return false;
    }
    
    // Decode all signals using CanCodec::decode_to_map
    can::SignalMap signals = can::CanCodec::decode_to_map(frame_def_, frame);
    
    // Map signals to ActuatorCmd structure
    // Signal names MUST match can_map.csv exactly!
    
    if (signals.count("system_enable")) {
        out_cmd.system_enable = (signals["system_enable"] > 0.5);
    }
    
    if (signals.count("mode")) {
        out_cmd.mode = static_cast<uint8_t>(signals["mode"]);
    }
    
    if (signals.count("steer_cmd_deg")) {
        out_cmd.steer_cmd_deg = signals["steer_cmd_deg"];
    }
    
    if (signals.count("drive_torque_cmd_nm")) {
        out_cmd.drive_torque_cmd_nm = signals["drive_torque_cmd_nm"];
    }
    
    if (signals.count("brake_cmd_pct")) {
        out_cmd.brake_cmd_pct = signals["brake_cmd_pct"];
    }
    
    // Update timestamp
    out_cmd.last_update_t_s = sim_time;
    
    LOG_DEBUG("CAN RX decoded 0x%03X: enable=%d mode=%d torque=%.1fNm brake=%.1f%% steer=%.1f°",
              frame.can_id,
              out_cmd.system_enable, 
              out_cmd.mode,
              out_cmd.drive_torque_cmd_nm,
              out_cmd.brake_cmd_pct,
              out_cmd.steer_cmd_deg);
    
    return true;
}

} // namespace can