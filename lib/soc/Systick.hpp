#pragma once

/*
 *   Name: Systick.hpp
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

    struct Tick_callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    void enable(std::uint32_t a_start_value, Prescaler a_prescaler, std::uint32_t a_priority);
    void disable();

    void register_tick_callback(const Tick_callback& a_callback);
    void unregister_tick_callback();

    bool is_enabled();

private:
    Tick_callback tick_callback;

private:
    void friend systick_interrupt_handler(Systick* a_p_this);
};
} // namespace soc