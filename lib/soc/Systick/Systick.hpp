#pragma once

/*
 *   Name: Systick.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>
#include <utility>

// externals
#ifdef STM32L4
#include <stm32l4xx.h>
#endif

// soc
#include <soc/Factory.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
class Systick : private cml::Non_copyable
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = SysTick_CTRL_CLKSOURCE_Msk,
        _8 = 0
    };

    Systick()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
    }

    ~Systick();

    void enable(std::uint32_t a_start_value, Prescaler a_prescaler);
    void disable();

    bool is_enabled();

    std::uint32_t get_idx()
    {
        return this->idx;
    }

    operator SysTick_Type*()
    {
        return SysTick;
    }

    operator const SysTick_Type*() const
    {
        return SysTick;
    }

private:
    Systick(std::uint32_t a_idx)
        : idx(a_idx)
    {
    }

    const std::uint32_t idx;

    template<typename Periph_t, std::size_t id> friend class Factory;
};

template<> class Factory<Systick> : private cml::Non_constructible
{
public:
    static Systick create()
    {
        return Systick(0);
    }
};
} // namespace soc