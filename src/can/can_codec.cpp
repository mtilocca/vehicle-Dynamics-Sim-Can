#include "can/can_codec.hpp"

#include <cmath>
#include <cstring>

#include "utils/bitpack.hpp"

namespace can {

bool CanCodec::has(const SignalMap& m, const std::string& key) {
    return m.find(key) != m.end();
}

double CanCodec::get_or(const SignalMap& m, const std::string& key, double fallback) {
    auto it = m.find(key);
    return (it == m.end()) ? fallback : it->second;
}

double CanCodec::clamp(double v, double lo, double hi) {
    if (lo > hi) return v;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int64_t CanCodec::round_to_i64(double v) {
    // symmetric rounding
    return static_cast<int64_t>(std::llround(v));
}

SignalMap CanCodec::decode_to_map(const FrameDef& def, const struct can_frame& frame) {
    SignalMap out;

    const int dlc = static_cast<int>(frame.can_dlc);
    for (const auto& sig : def.signals) {
        uint64_t raw_u = utils::get_bits_lsb0(
            frame.data, dlc,
            sig.start_bit, sig.bit_length,
            sig.endianness
        );

        double eng = 0.0;
        if (sig.is_signed) {
            int64_t raw_s = utils::sign_extend(raw_u, sig.bit_length);
            eng = static_cast<double>(raw_s) * sig.factor + sig.offset;
        } else {
            eng = static_cast<double>(raw_u) * sig.factor + sig.offset;
        }

        out[sig.signal_name] = eng;
    }

    return out;
}

void CanCodec::encode_from_map(const FrameDef& def, const SignalMap& values, struct can_frame& out) {
    std::memset(&out, 0, sizeof(out));
    out.can_id  = def.frame_id;
    out.can_dlc = static_cast<__u8>(def.dlc);

    // Ensure bytes are zeroed
    std::memset(out.data, 0, sizeof(out.data));

    for (const auto& sig : def.signals) {
        double eng = get_or(values, sig.signal_name, sig.default_value);
        eng = clamp(eng, sig.min, sig.max);

        // raw = (eng - offset) / factor
        double raw_f = (eng - sig.offset) / sig.factor;

        uint64_t raw_u = 0;
        if (sig.is_signed) {
            int64_t raw_s = round_to_i64(raw_f);

            // mask to bit_length two's complement form
            if (sig.bit_length < 64) {
                const uint64_t mask = (1ULL << sig.bit_length) - 1ULL;
                raw_u = static_cast<uint64_t>(raw_s) & mask;
            } else {
                raw_u = static_cast<uint64_t>(raw_s);
            }
        } else {
            uint64_t ru = static_cast<uint64_t>(raw_f < 0 ? 0 : round_to_i64(raw_f));
            if (sig.bit_length < 64) {
                const uint64_t mask = (1ULL << sig.bit_length) - 1ULL;
                raw_u = ru & mask;
            } else {
                raw_u = ru;
            }
        }

        utils::set_bits_lsb0(
            out.data, static_cast<size_t>(out.can_dlc),
            sig.start_bit, sig.bit_length,
            sig.endianness,
            raw_u
        );
    }
}

} // namespace can