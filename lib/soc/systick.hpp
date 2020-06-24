#pragma once

/*
    Name: systick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/integer.hpp>
#include <cml/time.hpp>
#include <cml/debug/assert.hpp>

namespace soc {

class systick
{
public:

    struct Tick_callback
    {
        using Function = void(*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    static void enable(cml::uint32 a_start_value, cml::uint32 a_priority);
    static void disable();

    static void register_tick_callback(const Tick_callback& a_callback);
    static void unregister_tick_callback();

    static bool is_enabled();

private:

    systick()               = delete;
    systick(systick&&)      = delete;
    systick(const systick&) = delete;
    ~systick()              = default;

    systick& operator = (systick&&)      = delete;
    systick& operator = (const systick&) = delete;
};

} // namespace soc