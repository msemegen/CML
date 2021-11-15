#pragma once

/*
 *   Name: polling.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#ifdef STM32L4
#include <stm32l4xx.h>
#endif

// soc
#include <soc/Factory.hpp>
#include <soc/Systick/Systick.hpp>
#ifdef STM32L4
#include <soc/m4/stm32l4/Polling.hpp>
#endif

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
template<> class m4::stm32l4::Polling<Systick> : private cml::Non_copyable
{
public:
    Polling()
        : p_systick(nullptr)
    {
    }

    std::uint32_t get_value() const
    {
        return static_cast<SysTick_Type*>(*(this->p_systick))->VAL;
    }

    Systick* get_handle()
    {
        return this->p_systick;
    }

    const Systick* get_handle() const
    {
        return this->p_systick;
    }

private:
    Polling(Systick* a_p_systick)
        : p_systick(a_p_systick)
    {
    }

    Systick* p_systick;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<> class Factory<m4::stm32l4::Polling<Systick>> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<Systick> create(Systick* a_p_systick)
    {
        return m4::stm32l4::Polling<Systick>(a_p_systick);
    }
};
} // namespace soc