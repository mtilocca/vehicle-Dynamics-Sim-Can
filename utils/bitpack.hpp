#pragma once
#include <cstdint>
#include <vector>

namespace utils
{

    // Bit numbering convention: LSB0
    // - start_bit = 0 means LSB of byte[0]
    // - start_bit = 7 means MSB of byte[0]
    // - start_bit = 8 means LSB of byte[1]
    // etc.
    enum class Endianness
    {
        Little, // Intel-style for multi-byte fields
        Big     // Motorola-style (implemented as bit-reversal across bytes for the field)
    };

    // Read up to 64 bits into uint64_t
    uint64_t get_bits_lsb0(const uint8_t *data, size_t data_len,
                           int start_bit, int bit_length, Endianness e);

    // Write up to 64 bits from uint64_t
    bool set_bits_lsb0(uint8_t *data, size_t data_len,
                       int start_bit, int bit_length, Endianness e,
                       uint64_t value);

    // Helpers for signed conversion
    int64_t sign_extend(uint64_t v, int bit_length);

} // namespace utils