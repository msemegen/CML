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
#include <cml/utils/wait_until.hpp>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

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
        cml::utils::wait_until::all_bits(&(PWR->SR2), PWR_SR2_VOSF_Pos, true);
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

template<> class rcc<pwr>
{
public:
    static void enable()
    {
        cml::bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_PWREN);
    }

    static void disable()
    {
        cml::bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_PWREN);
    }

private:
    rcc()           = delete;
    rcc(const rcc&) = delete;
    rcc(rcc&&)      = delete;
    ~rcc()          = delete;

    rcc& operator=(const rcc&) = delete;
    rcc& operator=(rcc&&) = delete;
};

} // namespace stm32l4
} // namespace m4
} // namespace soc