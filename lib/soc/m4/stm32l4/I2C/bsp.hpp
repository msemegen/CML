#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/defs.hpp>
#include <soc/m4/stm32l4/I2C/I2C.hpp>
#include <soc/m4/stm32l4/I2C/DMA.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
#if defined(SOC_I2C1_PRESENT)
template<> class Peripheral<m4::stm32l4::I2C_master, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_master create()
    {
        return m4::stm32l4::I2C_master(0, I2C1, IRQn_Type::I2C1_EV_IRQn, IRQn_Type::I2C1_ER_IRQn);
    }
};

template<> class Peripheral<m4::stm32l4::I2C_slave, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_slave create()
    {
        return m4::stm32l4::I2C_slave(0, I2C1, IRQn_Type::I2C1_EV_IRQn, IRQn_Type::I2C1_ER_IRQn);
    }
};
#endif

#if defined(SOC_I2C2_PRESENT)
template<> class Peripheral<m4::stm32l4::I2C_master, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_master create()
    {
        return m4::stm32l4::I2C_master(1, I2C2, IRQn_Type::I2C2_EV_IRQn, IRQn_Type::I2C2_ER_IRQn);
    }
};

template<> class Peripheral<m4::stm32l4::I2C_slave, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_slave create()
    {
        return m4::stm32l4::I2C_slave(1, I2C2, IRQn_Type::I2C2_EV_IRQn, IRQn_Type::I2C2_ER_IRQn);
    }
};
#endif

#if defined(SOC_I2C3_PRESENT)
template<> class Peripheral<m4::stm32l4::I2C_master, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_master create()
    {
        return m4::stm32l4::I2C_master(2, I2C3, IRQn_Type::I2C3_EV_IRQn, IRQn_Type::I2C3_ER_IRQn);
    }
};

template<> class Peripheral<m4::stm32l4::I2C_slave, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_slave create()
    {
        return m4::stm32l4::I2C_slave(2, I2C3, IRQn_Type::I2C3_EV_IRQn, IRQn_Type::I2C3_ER_IRQn);
    }
};
#endif

#if defined(SOC_I2C4_PRESENT)
template<> class Peripheral<m4::stm32l4::I2C_master, 4u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_master create()
    {
        return m4::stm32l4::I2C_master(3, I2C4, IRQn_Type::I2C4_EV_IRQn, IRQn_Type::I2C4_ER_IRQn);
    }
};

template<> class Peripheral<m4::stm32l4::I2C_slave, 4u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::I2C_slave create()
    {
        return m4::stm32l4::I2C_slave(3, I2C4, IRQn_Type::I2C4_EV_IRQn, IRQn_Type::I2C4_ER_IRQn);
    }
};
#endif
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {
#if defined(SOC_I2C1_PRESENT)
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 1>::disable();
#endif

#if defined(SOC_I2C2_PRESENT)
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 2>::disable();
#endif

#if defined(SOC_I2C3_PRESENT)
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 3>::disable();
#endif

#if defined(SOC_I2C4_PRESENT)
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 4>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc