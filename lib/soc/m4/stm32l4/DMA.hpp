#pragma once

/*
 *   Name: DMA.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#undef DMA1
#undef DMA2

template<typename Periph_t> class DMA_base : private cml::Non_constructible
{
};

template<typename Periph_t = void> class DMA1 : private cml::Non_copyable
{
public:
    enum class Priority : std::uint32_t
    {
        low       = 0x0u,
        medium    = DMA_CCR_PL_0,
        high      = DMA_CCR_PL_1,
        very_high = DMA_CCR_PL_0 | DMA_CCR_PL_1
    };
};

template<typename Periph_t = void> class DMA2 : private cml::Non_copyable
{
public:
    enum class Priority : std::uint32_t
    {
        low       = 0x0u,
        medium    = DMA_CCR_PL_0,
        high      = DMA_CCR_PL_1,
        very_high = DMA_CCR_PL_0 | DMA_CCR_PL_1
    };
};

template<> class rcc<DMA1<>>
{
public:
    void enable(bool a_enable_in_lp);
    void disable();
};

template<> class rcc<DMA2<>>
{
public:
    void enable(bool a_enable_in_lp);
    void disable();
};
} // namespace stm32l4
} // namespace m4
} // namespace soc