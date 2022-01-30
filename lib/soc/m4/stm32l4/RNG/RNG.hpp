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
    class Polling : private cml::Non_copyable
    {
    public:
        Polling(Polling&&) = default;
        Polling& operator=(Polling&&) = default;

        bool get_value(std::uint32_t* a_p_value, std::uint32_t a_timeout);

        bool is_created() const
        {
            return nullptr != this->p_RNG;
        }

    private:
        Polling()
            : p_RNG(nullptr)
        {
        }

        Polling(RNG* a_p_RNG)
            : p_RNG(a_p_RNG)
        {
        }

        RNG* p_RNG;

        friend RNG;
    };
    class Interrupt : private cml::Non_copyable
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

        Interrupt(Interrupt&&) = default;
        Interrupt& operator=(Interrupt&&) = default;

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

        bool is_created() const
        {
            return nullptr != this->p_RNG;
        }

    private:
        Interrupt()
            : p_RNG(nullptr)
        {
        }

        Interrupt(RNG* a_p_RNG)
            : p_RNG(a_p_RNG)
        {
        }

        RNG* p_RNG;
        Callback callback;

        friend void RNG_interrupt_handler();
        friend RNG;
    };

    RNG(RNG&&)   = default;
    RNG& operator=(RNG&&) = default;

    RNG()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
    }

    ~RNG()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    bool enable(std::uint32_t a_timeout);
    void disable();

    std::uint32_t get_idx()
    {
        return this->idx;
    }

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
    }

    std::uint32_t idx;
    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

template<> class rcc<RNG> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};

void RNG_interrupt_handler();
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