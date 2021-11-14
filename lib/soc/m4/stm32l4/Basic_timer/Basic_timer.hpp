#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <tuple>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class Basic_timer : private cml::Non_copyable
{
public:
    struct Enable_config
    {
        enum class Mode : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Autoreload_preload : std::uint32_t
        {
            disabled = 0x0u,
            enabled  = TIM_CR1_ARPE,
        };

        Mode mode                              = cml::various::get_enum_incorrect_value<Mode>();
        Autoreload_preload auto_reload_preload = cml::various::get_enum_incorrect_value<Autoreload_preload>();

        std::uint16_t prescaler   = 0u;
        std::uint16_t auto_reload = 0u;
    };

    ~Basic_timer()
    {
        this->stop();
        this->disable();
    }

    void enable(const Enable_config& a_config);
    void disable();

    void start();
    void stop();

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    operator TIM_TypeDef*()
    {
        return this->p_registers;
    }

    operator const TIM_TypeDef*() const
    {
        return this->p_registers;
    }

private:
    Basic_timer(std::size_t a_idx, TIM_TypeDef* a_p_registers)
        : idx(0u)
        , p_registers(a_p_registers)
    {
    }

    const std::uint32_t idx;
    TIM_TypeDef* p_registers;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<std::size_t id> class rcc<Basic_timer, id> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp) = delete;
    static void disable()                   = delete;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc
