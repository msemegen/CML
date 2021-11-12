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

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<RNG> : private cml::Non_copyable
{
public:
    struct New_value_callback
    {
        using Function =
            void (*)(std::uint32_t a_value, bool a_clock_error, bool a_seed_error, RNG* p_rng, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Interrupt(RNG* a_p_RNG);
    ~Interrupt();

    void enable(const IRQ_config& a_irq_config);
    void disable();

    void register_callback(const New_value_callback& a_callback);

private:
    RNG* p_RNG;

    New_value_callback new_value_callback;

private:
    friend void RNG_interrupt_handler();
};

} // namespace stm32l4
} // namespace m4
} // namespace soc