#pragma once

/*
 *   Name: DMA.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// std
#include <cstdint>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<typename Periph_t = void*> class DMA : private cml::Non_copyable
{
public:
    enum class Priority : std::uint32_t
    {
        very_high = DMA_CCR_PL_0 | DMA_CCR_PL_1,
        high      = DMA_CCR_PL_1,
        medium    = DMA_CCR_PL_0,
        low       = 0x0u
    };
    enum class Mode : std::uint32_t
    {
        single   = 0x0u,
        circular = DMA_CCR_CIRC
    };
    enum class Event_flag : std::uint32_t
    {
        none              = 0x0u,
        half_transfer     = DMA_CCR_HTIE,
        transfer_complete = DMA_CCR_TCIE,
        transfer_error    = DMA_CCR_TEIE
    };

    struct Callback
    {
        using Function = void (*)(Event_flag a_event, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };
};

constexpr DMA<>::Event_flag operator|(DMA<>::Event_flag a_f1, DMA<>::Event_flag a_f2)
{
    return static_cast<DMA<>::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr DMA<>::Event_flag operator&(DMA<>::Event_flag a_f1, DMA<>::Event_flag a_f2)
{
    return static_cast<DMA<>::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr DMA<>::Event_flag operator|=(DMA<>::Event_flag& a_f1, DMA<>::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

template<> class rcc<DMA<>, 1> : private cml::Non_constructible
{
public:
    static void enable()
    {
        cml::bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA1EN);
    }
    static void disable()
    {
        cml::bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA1EN);
    }
};

template<> class rcc<DMA<>, 2> : private cml::Non_constructible
{
public:
    static void enable()
    {
        cml::bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA2EN);
    }
    static void disable()
    {
        cml::bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA2EN);
    }
};
} // namespace stm32l4
} // namespace m4
} // namespace soc