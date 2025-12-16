#include "can_map.hpp"

#include <algorithm>

namespace can {

utils::Endianness CanMap::parse_endianness(const std::string& s) {
    if (s == "big" || s == "BIG")
        return utils::Endianness::Big;
    return utils::Endianness::Little;
}

bool CanMap::load(const std::string& csv_path) {
    utils::CsvReader csv;
    if (!csv.open(csv_path))
        return false;

    rx_frames_.clear();
    tx_frames_.clear();

    std::vector<std::string> row;
    while (csv.read_row(row)) {
        const std::string direction = csv.get(row, "direction");
        const uint32_t frame_id =
            utils::CsvReader::to_uint32(csv.get(row, "frame_id"));
        const std::string frame_name = csv.get(row, "frame_name");
        const int cycle_ms =
            utils::CsvReader::to_int(csv.get(row, "cycle_ms"));
        const int dlc =
            utils::CsvReader::to_int(csv.get(row, "dlc"), 8);

        SignalRule sig;
        sig.signal_name = csv.get(row, "signal_name");
        sig.target = csv.get(row, "target");
        sig.start_bit =
            utils::CsvReader::to_int(csv.get(row, "start_bit"));
        sig.bit_length =
            utils::CsvReader::to_int(csv.get(row, "bit_length"));
        sig.endianness =
            parse_endianness(csv.get(row, "endianness"));
        sig.is_signed =
            utils::CsvReader::to_bool(csv.get(row, "signed"));
        sig.factor =
            utils::CsvReader::to_double(csv.get(row, "factor"), 1.0);
        sig.offset =
            utils::CsvReader::to_double(csv.get(row, "offset"), 0.0);
        sig.min =
            utils::CsvReader::to_double(csv.get(row, "min"), 0.0);
        sig.max =
            utils::CsvReader::to_double(csv.get(row, "max"), 0.0);
        sig.default_value =
            utils::CsvReader::to_double(csv.get(row, "default"), 0.0);
        sig.unit = csv.get(row, "unit");

        if (direction == "rx") {
            auto& frame = rx_frames_[frame_id];
            if (frame.signals.empty()) {
                frame.frame_id = frame_id;
                frame.frame_name = frame_name;
                frame.cycle_ms = cycle_ms;
                frame.dlc = dlc;
            }
            frame.signals.push_back(sig);
        }
        else if (direction == "tx") {
            auto it = std::find_if(
                tx_frames_.begin(), tx_frames_.end(),
                [&](const FrameDef& f) { return f.frame_id == frame_id; });

            if (it == tx_frames_.end()) {
                FrameDef frame;
                frame.frame_id = frame_id;
                frame.frame_name = frame_name;
                frame.cycle_ms = cycle_ms;
                frame.dlc = dlc;
                frame.signals.push_back(sig);
                tx_frames_.push_back(frame);
            } else {
                it->signals.push_back(sig);
            }
        }
    }

    // Sort TX frames by cycle time (nice for schedulers)
    std::sort(tx_frames_.begin(), tx_frames_.end(),
              [](const FrameDef& a, const FrameDef& b) {
                  return a.cycle_ms < b.cycle_ms;
              });

    return true;
}

const FrameDef* CanMap::find_rx_frame(uint32_t frame_id) const {
    auto it = rx_frames_.find(frame_id);
    if (it == rx_frames_.end())
        return nullptr;
    return &it->second;
}

} // namespace can