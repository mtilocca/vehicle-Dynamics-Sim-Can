#pragma once
// CAN TX bit-packing helpers — no Zephyr deps, host-buildable for unit tests.
// Included by can_tx.cpp (firmware) and test/zephyr_portability/ (host tests).
#include <stdint.h>
#include <cmath>

namespace can_codec {

inline double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Little-endian pack into a CAN payload byte array.
inline void pack_i16(uint8_t* d, int byte, int16_t v)
{
    d[byte]   = (uint8_t)( v        & 0xFF);
    d[byte+1] = (uint8_t)((v >> 8)  & 0xFF);
}
inline void pack_u16(uint8_t* d, int byte, uint16_t v)
{
    d[byte]   = (uint8_t)( v        & 0xFF);
    d[byte+1] = (uint8_t)((v >> 8)  & 0xFF);
}
inline void pack_i32(uint8_t* d, int byte, int32_t v)
{
    d[byte]   = (uint8_t)( v        & 0xFF);
    d[byte+1] = (uint8_t)((v >> 8)  & 0xFF);
    d[byte+2] = (uint8_t)((v >> 16) & 0xFF);
    d[byte+3] = (uint8_t)((v >> 24) & 0xFF);
}
inline void pack_u32(uint8_t* d, int byte, uint32_t v)
{
    d[byte]   = (uint8_t)( v        & 0xFF);
    d[byte+1] = (uint8_t)((v >> 8)  & 0xFF);
    d[byte+2] = (uint8_t)((v >> 16) & 0xFF);
    d[byte+3] = (uint8_t)((v >> 24) & 0xFF);
}

// Physical value → raw integer (nearest integer, clamped).
inline int16_t  enc_i16(double phys, double factor)
{
    return (int16_t)(int64_t)std::round(clamp(phys / factor, -32768.0, 32767.0));
}
inline uint16_t enc_u16(double phys, double factor)
{
    return (uint16_t)(int64_t)std::round(clamp(phys / factor, 0.0, 65535.0));
}
inline int32_t  enc_i32(double phys, double factor)
{
    return (int32_t)(int64_t)std::round(clamp(phys / factor, -2147483648.0, 2147483647.0));
}
inline uint32_t enc_u32(double phys, double factor)
{
    return (uint32_t)(int64_t)std::round(clamp(phys / factor, 0.0, 4294967295.0));
}

} // namespace can_codec
