#pragma once

/*
    Name: rng.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//external
#include <stm32l452xx.h>

//cml
#include <cml/bit.hpp>
#include <cml/time.hpp>

namespace soc {
namespace stm32l452xx {
namespace system {

class rng
{
public:

    struct New_value_callback
    {
        using Function = void(*)(cml::uint32 a_value, bool a_clock_error, bool a_seed_error, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    rng()           = delete;
    rng(rng&&)      = delete;
    rng(const rng&) = delete;

    rng& operator = (rng&&)      = delete;
    rng& operator = (const rng&) = delete;

    static bool enable(cml::uint32 a_irq_priority, cml::time::tick a_timeout);
    static void disable();

    static bool get_value_polling(cml::uint32* a_p_value, cml::time::tick a_timeout);

    static void get_value_it(const New_value_callback& a_callback);
};

} // namespace system
} // namespace stm32l452xx
} // namespace soc