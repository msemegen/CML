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
#include <soc/m4/stm32l4/WWDG/WWDG.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<WWDG> : private cml::Non_copyable
{
public:
    struct Callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    Interrupt()
        : p_WWDG(nullptr)
    {
    }

    ~Interrupt()
    {
        this->disable();
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    void register_callback(const Callback& a_callback);
    void unregister_callback();

private:
    WWDG* p_WWDG;
    Callback callback;

    friend void WWDG_interrupt_handler();
    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Factory<m4::stm32l4::Interrupt<m4::stm32l4::WWDG>> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::WWDG> create()
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::WWDG>();
    }
};
} // namespace soc