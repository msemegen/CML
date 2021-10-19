#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/IRQ.hpp>
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
    struct Early_wakeup_callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Interrupt();
    ~Interrupt();

    void enable(const IRQ& a_irq);
    void disable();

    void register_callback(const Early_wakeup_callback& a_callback);

private:
    Early_wakeup_callback early_wakeup_callback;

private:
    friend void WWDG_interrupt_handler(Interrupt<WWDG>* a_p_this);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc