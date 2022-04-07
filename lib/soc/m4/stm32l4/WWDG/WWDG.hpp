#pragma once

/*
 *   Name: WWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#ifdef WWDG
#undef WWDG
#endif

class WWDG : private cml::Non_copyable
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0,
        _2 = WWDG_CFR_WDGTB_0,
        _4 = WWDG_CFR_WDGTB_1,
        _8 = WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1
    };

    class Interrupt : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        void register_callback(const Callback& a_callback);
        void unregister_callback();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(IRQn_Type::WWDG_IRQn);
        }

        bool is_created() const
        {
            return nullptr != this->p_WWDG;
        }

    private:
        WWDG* p_WWDG;
        friend WWDG;
    };

    WWDG()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
        this->interrupt.p_WWDG = nullptr;
    }

    void enable(Prescaler a_prescaler, std::uint16_t a_reload, std::uint16_t a_window);
    void feed();

    bool is_enabled() const;

    operator WWDG_TypeDef*()
    {
        return reinterpret_cast<WWDG_TypeDef*>(WWDG_BASE);
    }

    operator const WWDG_TypeDef*() const
    {
        return reinterpret_cast<WWDG_TypeDef*>(WWDG_BASE);
    }

    Interrupt interrupt;

private:
    WWDG(std::uint32_t a_idx)
        : idx(a_idx)
    {
        this->interrupt.p_WWDG = this;
    }

    std::uint32_t idx;

    std::uint16_t reload;
    Interrupt::Callback callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
    friend void WWDG_interrupt_handler();
};
void WWDG_interrupt_handler();

} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Peripheral<m4::stm32l4::WWDG> : private cml::Non_constructible
{
public:
    static m4::stm32l4::WWDG create()
    {
        return m4::stm32l4::WWDG(0);
    }
};
} // namespace soc