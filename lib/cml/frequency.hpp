#pragma once

/*
    Name: frequency.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace cml {

using frequency = uint32_t;

constexpr static inline frequency Hz(uint32_t a_freq_hz)
{
    return a_freq_hz;
}

constexpr static inline frequency kHz(uint32_t a_freq_kHz)
{
    return a_freq_kHz * 1000u;
}

constexpr static inline frequency MHz(uint32_t a_freq_MHz)
{
    return a_freq_MHz * 1000000u;
}

} // namespace cml