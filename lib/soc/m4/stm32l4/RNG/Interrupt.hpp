#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>
#include <soc/m4/stm32l4/RNG/RNG.hpp>

// cml
#include <cml/Non_constructible.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<RNG> : private cml::Non_constructible
{
public:
    struct Callback
    {
        using Function = void (*)(std::uint32_t a_value, bool a_clock_error, bool a_seed_error, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    static void enable(const IRQ_config& a_irq_config);
    static void disable();

    static void register_callback(const Callback& a_callback);
    static void unregister_callback();

private:
    static Callback callback;

    friend void RNG_interrupt_handler();
};
} // namespace stm32l4
} // namespace m4
} // namespace soc