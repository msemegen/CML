#pragma once

/*
    Name: systick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// externals
#ifdef STM32L452xx
#include <stm32l4xx.h>
#endif

#ifdef STM32L011xx
#include <stm32l0xx.h>
#endif

// cml
#include <cml/debug/assert.hpp>
#include <cml/time.hpp>

namespace soc {

class systick
{
public:
    enum class Prescaler : uint32_t
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

    static void enable(uint32_t a_start_value, Prescaler a_prescaler, uint32_t a_priority);
    static void disable();

    static void register_tick_callback(const Tick_callback& a_callback);
    static void unregister_tick_callback();

    static bool is_enabled();

private:
    systick()               = delete;
    systick(systick&&)      = delete;
    systick(const systick&) = delete;
    ~systick()              = default;

    systick& operator=(systick&&) = delete;
    systick& operator=(const systick&) = delete;
};

} // namespace soc