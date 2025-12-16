#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/bitpack.hpp"

namespace can {

// One signal inside a CAN frame
struct SignalRule {
    std::string signal_name;
    std::string target;          // actuator_cmd / sensor_out

    int start_bit = 0;
    int bit_length = 0;

    utils::Endianness endianness = utils::Endianness::Little;
    bool is_signed = false;

    double factor = 1.0;
    double offset = 0.0;
    double min = 0.0;
    double max = 0.0;
    double default_value = 0.0;

    std::string unit;
};

// One CAN frame definition
struct FrameDef {
    uint32_t frame_id = 0;
    std::string frame_name;
    int cycle_ms = 0;
    int dlc = 8;

    std::vector<SignalRule> signals;
};

// CAN map loaded from CSV
class CanMap {
public:
    bool load(const std::string& csv_path);

    // RX: frame_id → frame definition (signals to decode)
    const std::unordered_map<uint32_t, FrameDef>& rx_frames() const { return rx_frames_; }

    // TX: ordered list of frames to transmit
    const std::vector<FrameDef>& tx_frames() const { return tx_frames_; }

    // Lookup helpers
    const FrameDef* find_rx_frame(uint32_t frame_id) const;
    const FrameDef* find_tx_frame(uint32_t frame_id) const;

private:
    std::unordered_map<uint32_t, FrameDef> rx_frames_;
    std::vector<FrameDef> tx_frames_;

    static utils::Endianness parse_endianness(const std::string& s);
};

} // namespace can