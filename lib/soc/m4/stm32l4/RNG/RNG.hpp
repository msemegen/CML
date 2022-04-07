#pragma once

/*
 *   Name: RNG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

// external
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#ifdef RNG
#undef RNG
#endif
#define RNG_T ((RNG_TypeDef*)RNG_BASE)

class RNG : private cml::Non_copyable
{
public:
    class Polling
    {
    public:
        bool get_value(std::uint32_t* a_p_value, cml::Milliseconds a_timeout);

        bool is_created() const
        {
            return nullptr != this->p_RNG;
        }

    private:
        RNG* p_RNG;
        friend RNG;
    };
    class Interrupt
    {
    public:
        struct Callback
        {
            using Function = void (*)(std::uint32_t a_value,
                                      bool a_clock_error,
                                      bool a_seed_error,
                                      void* a_p_user_data);

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
            return 0u != NVIC_GetEnableIRQ(IRQn_Type::RNG_IRQn);
        }

    private:
        RNG* p_RNG = nullptr;
        friend RNG;
    };

    RNG(RNG&&)   = default;
    RNG& operator=(RNG&&) = default;

    RNG()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
        this->polling.p_RNG   = nullptr;
        this->interrupt.p_RNG = nullptr;
    }

    ~RNG()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    bool enable(cml::Milliseconds a_timeout);
    void disable();

    bool is_enabled() const
    {
        return cml::bit_flag::is(RNG_T->CR, RNG_CR_RNGEN);
    }

    bool is_created() const
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx;
    }

    operator RNG_TypeDef*()
    {
        return reinterpret_cast<RNG_TypeDef*>(RNG_BASE);
    }

    operator const RNG_TypeDef*() const
    {
        return reinterpret_cast<RNG_TypeDef*>(RNG_BASE);
    }

    Polling polling;
    Interrupt interrupt;

private:
    RNG(std::uint32_t a_idx)
        : idx(a_idx)
    {
        this->polling.p_RNG   = this;
        this->interrupt.p_RNG = this;
    }

    std::uint32_t idx;

    Interrupt::Callback callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
    friend void RNG_interrupt_handler();
};
void RNG_interrupt_handler();

template<> class rcc<RNG> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};

} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Peripheral<m4::stm32l4::RNG> : private cml::Non_constructible
{
public:
    static m4::stm32l4::RNG create()
    {
        return m4::stm32l4::RNG(0);
    }
};
} // namespace soc