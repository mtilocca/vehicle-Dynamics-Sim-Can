#include "bitpack.hpp"

namespace utils
{

    static inline bool in_range_bits(size_t data_len, int start_bit, int bit_length)
    {
        if (data_len == 0)
            return false;
        if (bit_length <= 0 || bit_length > 64)
            return false;
        if (start_bit < 0)
            return false;
        const int end_bit = start_bit + bit_length - 1;
        const int max_bit = static_cast<int>(data_len * 8) - 1;
        return end_bit <= max_bit;
    }

    static inline uint8_t get_bit_lsb0(const uint8_t *data, int bit_index)
    {
        const int byte_i = bit_index / 8;
        const int bit_i = bit_index % 8; // 0 = LSB
        return (data[byte_i] >> bit_i) & 0x1u;
    }

    static inline void set_bit_lsb0(uint8_t *data, int bit_index, uint8_t bit_val)
    {
        const int byte_i = bit_index / 8;
        const int bit_i = bit_index % 8;
        const uint8_t mask = static_cast<uint8_t>(1u << bit_i);
        if (bit_val)
            data[byte_i] |= mask;
        else
            data[byte_i] &= static_cast<uint8_t>(~mask);
    }

    // For "Big" endianness, we interpret the field bits in reverse order (MSB-first)
    // within the bit_length window. This keeps the API simple and deterministic,
    // even if you don't use Big-endian in V1.
    static inline int map_field_bit_index(int start_bit, int bit_length, Endianness e, int k /*0..bit_length-1*/)
    {
        if (e == Endianness::Little)
        {
            // field bit k is at absolute bit start_bit + k
            return start_bit + k;
        }
        // Big: treat field bit 0 as MSB of the field, mapped to highest absolute bit
        // and field bit (bit_length-1) as LSB mapped to start_bit.
        // So absolute bit index = start_bit + (bit_length - 1 - k)
        return start_bit + (bit_length - 1 - k);
    }

    uint64_t get_bits_lsb0(const uint8_t *data, size_t data_len,
                           int start_bit, int bit_length, Endianness e)
    {
        if (!in_range_bits(data_len, start_bit, bit_length))
            return 0;

        uint64_t out = 0;
        for (int k = 0; k < bit_length; ++k)
        {
            const int abs_bit = map_field_bit_index(start_bit, bit_length, e, k);
            const uint64_t b = static_cast<uint64_t>(get_bit_lsb0(data, abs_bit));
            // out bit k is always LSB-first in returned integer
            out |= (b << k);
        }
        return out;
    }

    bool set_bits_lsb0(uint8_t *data, size_t data_len,
                       int start_bit, int bit_length, Endianness e,
                       uint64_t value)
    {
        if (!in_range_bits(data_len, start_bit, bit_length))
            return false;

        // mask value to bit_length
        if (bit_length < 64)
        {
            const uint64_t mask = (1ULL << bit_length) - 1ULL;
            value &= mask;
        }

        for (int k = 0; k < bit_length; ++k)
        {
            const int abs_bit = map_field_bit_index(start_bit, bit_length, e, k);
            const uint8_t b = static_cast<uint8_t>((value >> k) & 0x1ULL);
            set_bit_lsb0(data, abs_bit, b);
        }
        return true;
    }

    int64_t sign_extend(uint64_t v, int bit_length)
    {
        if (bit_length <= 0 || bit_length > 64)
            return static_cast<int64_t>(v);
        if (bit_length == 64)
            return static_cast<int64_t>(v);

        const uint64_t sign_bit = 1ULL << (bit_length - 1);
        if (v & sign_bit)
        {
            const uint64_t mask = (~0ULL) << bit_length;
            v |= mask;
        }
        return static_cast<int64_t>(v);
    }

} // namespace utils