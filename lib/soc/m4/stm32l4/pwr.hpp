#pragma once

/*
 *   Name: pwr.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

class pwr
{
public:
    enum class Voltage_scaling : uint32_t
    {
        _1 = PWR_CR1_VOS_0,
        _2 = PWR_CR1_VOS_1,
    };

    static void set_voltage_scaling(Voltage_scaling a_scaling)
    {
        cml::bit_flag::set(&(PWR->CR1), PWR_CR1_VOS, static_cast<uint32_t>(a_scaling));

        while (true == cml::bit::is(PWR->SR2, PWR_SR2_VOSF_Pos))
            ;
    }

    static Voltage_scaling get_voltage_scaling()
    {
        return static_cast<Voltage_scaling>(cml::bit_flag::get(PWR->CR1, PWR_CR1_VOS));
    }

private:
    pwr()           = delete;
    pwr(const pwr&) = delete;
    pwr(pwr&&)      = delete;
    ~pwr()          = delete;

    pwr& operator=(const pwr&) = delete;
    pwr& operator=(pwr&&) = delete;
};

} // namespace stm32l4
} // namespace m4
} // namespace soc