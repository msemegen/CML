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

constexpr static inline frequency kHz_to_Hz(uint32_t a_freq)
{
    return a_freq * 1000u;
}

constexpr static inline frequency MHz_to_Hz(uint32_t a_freq)
{
    return a_freq * 1000000u;
}

constexpr static inline frequency Hz_to_kHz(uint32_t a_freq)
{
    return a_freq / 1000;
}

constexpr static inline frequency Hz_to_MHz(uint32_t a_freq)
{
    return a_freq / 1000000;
}

} // namespace cml