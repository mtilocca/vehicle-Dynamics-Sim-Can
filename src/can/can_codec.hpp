#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include <linux/can.h>

#include "can/can_map.hpp"

namespace can {

// Generic decoded values (engineering units) by signal name
using SignalMap = std::unordered_map<std::string, double>;

class CanCodec {
public:
    // Decode a Linux can_frame into engineering values for all signals in FrameDef
    static SignalMap decode_to_map(const FrameDef& def, const struct can_frame& frame);

    // Encode a Linux can_frame from engineering values (missing values use defaults)
    // - Clamps to [min,max]
    // - Applies factor/offset
    // - Packs bits using your LSB0 bitpack
    static void encode_from_map(const FrameDef& def, const SignalMap& values, struct can_frame& out);

    // Convenience
    static bool has(const SignalMap& m, const std::string& key);
    static double get_or(const SignalMap& m, const std::string& key, double fallback);

private:
    static double clamp(double v, double lo, double hi);
    static int64_t round_to_i64(double v);
};

} // namespace can