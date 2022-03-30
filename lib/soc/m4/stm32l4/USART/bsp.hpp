#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/USART/DMA.hpp>
#include <soc/m4/stm32l4/USART/RS485.hpp>
#include <soc/m4/stm32l4/USART/USART.hpp>
#include <soc/m4/stm32l4/defs.hpp>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
#if defined(SOC_USART1_PRESENT)
template<> class Peripheral<m4::stm32l4::USART, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::USART create()
    {
        return m4::stm32l4::USART(0u, USART1, IRQn_Type::USART1_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::USART, 1u, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::USART> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::USART>(0u,
                                                    USART1,
                                                    DMA1_CSELR,
                                                    DMA1_Channel4,
                                                    DMA1_Channel4_IRQn,
                                                    { 0x2000u, 0xF000u },
                                                    DMA1_Channel5,
                                                    DMA1_Channel5_IRQn,
                                                    { 0x20000u, 0xF0000u });
    }
};
template<> class Peripheral<m4::stm32l4::USART, 1u, m4::stm32l4::DMA<>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::USART> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::USART>(1u,
                                                    USART1,
                                                    DMA2_CSELR,
                                                    DMA2_Channel6,
                                                    DMA2_Channel6_IRQn,
                                                    { 0x200000u, 0xF00000u },
                                                    DMA2_Channel7,
                                                    DMA2_Channel7_IRQn,
                                                    { 0x2000000u, 0xF000000u });
    }
};

template<> class Peripheral<m4::stm32l4::RS485, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::RS485 create()
    {
        return m4::stm32l4::RS485(0u, USART1, IRQn_Type::USART1_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::RS485, 1u, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::RS485> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::RS485>(0u,
                                                    USART1,
                                                    DMA1_CSELR,
                                                    DMA1_Channel4,
                                                    DMA1_Channel4_IRQn,
                                                    { 0x2000u, 0xF000u },
                                                    DMA1_Channel5,
                                                    DMA1_Channel5_IRQn,
                                                    { 0x20000u, 0xF0000u });
    }
};
template<> class Peripheral<m4::stm32l4::RS485, 1u, m4::stm32l4::DMA<>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::RS485> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::RS485>(1u,
                                                    USART1,
                                                    DMA2_CSELR,
                                                    DMA2_Channel6,
                                                    DMA2_Channel6_IRQn,
                                                    { 0x200000u, 0xF00000u },
                                                    DMA2_Channel7,
                                                    DMA2_Channel7_IRQn,
                                                    { 0x2000000u, 0xF000000u });
    }
};
#endif

#if defined(SOC_USART2_PRESENT)
template<> class Peripheral<m4::stm32l4::USART, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::USART create()
    {
        return m4::stm32l4::USART(1u, USART2, IRQn_Type::USART2_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::USART, 2u, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::USART> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::USART>(0u,
                                                    USART2,
                                                    DMA1_CSELR,
                                                    DMA1_Channel7,
                                                    DMA1_Channel7_IRQn,
                                                    { 0x2000000u, 0xF000000u },
                                                    DMA1_Channel6,
                                                    DMA1_Channel6_IRQn,
                                                    { 0x200000u, 0xF00000u });
    }
};

template<> class Peripheral<m4::stm32l4::RS485, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::RS485 create()
    {
        return m4::stm32l4::RS485(1u, USART2, IRQn_Type::USART2_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::RS485, 2u, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::RS485> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::RS485>(0u,
                                                    USART2,
                                                    DMA1_CSELR,
                                                    DMA1_Channel7,
                                                    DMA1_Channel7_IRQn,
                                                    { 0x2000000u, 0xF000000u },
                                                    DMA1_Channel6,
                                                    DMA1_Channel6_IRQn,
                                                    { 0x200000u, 0xF00000u });
    }
};
#endif

#if defined(SOC_USART3_PRESENT)
template<> class Peripheral<m4::stm32l4::USART, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::USART create()
    {
        return m4::stm32l4::USART(2u, USART3, IRQn_Type::USART3_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::USART, 3u, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::USART> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::USART>(0,
                                                    USART2,
                                                    DMA1_CSELR,
                                                    DMA1_Channel2,
                                                    DMA1_Channel2_IRQn,
                                                    { 0x20u, 0xF0u },
                                                    DMA1_Channel3,
                                                    DMA1_Channel3_IRQn,
                                                    { 0x200u, 0xF00u });
    }
};

template<> class Peripheral<m4::stm32l4::RS485, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::RS485 create()
    {
        return m4::stm32l4::RS485(2u, USART3, IRQn_Type::USART3_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::RS485, 3u, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::RS485> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::RS485>(0,
                                                    USART2,
                                                    DMA1_CSELR,
                                                    DMA1_Channel2,
                                                    DMA1_Channel2_IRQn,
                                                    { 0x20u, 0xF0u },
                                                    DMA1_Channel3,
                                                    DMA1_Channel3_IRQn,
                                                    { 0x200u, 0xF00u });
    }
};
#endif
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {
#if defined(SOC_USART1_PRESENT)
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::HSI>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::PCLK1>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::SYSCLK>(bool a_enable_in_lp);
template<> void rcc<USART, 1u>::disable();
#endif

#if defined(SOC_USART2_PRESENT)
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::HSI>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::PCLK1>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::SYSCLK>(bool a_enable_in_lp);
template<> void rcc<USART, 2u>::disable();
#endif

#if defined(SOC_USART3_PRESENT)
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::HSI>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::PCLK1>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::SYSCLK>(bool a_enable_in_lp);
template<> void rcc<USART, 3>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc
