#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Factory.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>
#include <soc/m4/stm32l4/RNG/RNG.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<RNG> : private cml::Non_copyable
{
public:
    struct Callback
    {
        using Function = void (*)(std::uint32_t a_value, bool a_clock_error, bool a_seed_error, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    void enable(const IRQ_config& a_irq_config);
    void disable();

    void register_callback(const Callback& a_callback);
    void unregister_callback();

private:
    Interrupt() {}

    static Callback callback;

    friend void RNG_interrupt_handler();
    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Factory<m4::stm32l4::Interrupt<m4::stm32l4::RNG>> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::RNG> create()
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::RNG>();
    }
};
} // namespace soc