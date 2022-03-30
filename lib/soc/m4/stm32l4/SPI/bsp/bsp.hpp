#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/SPI/Interrupt.hpp>
#include <soc/m4/stm32l4/SPI/Polling.hpp>
#include <soc/m4/stm32l4/SPI/SPI.hpp>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/defs.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
#if defined(SOC_SPI1_PRESENT)
template<> class Peripheral<m4::stm32l4::SPI_master, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::SPI_master create()
    {
        return m4::stm32l4::SPI_master(0, SPI1);
    }
};
template<> class Peripheral<m4::stm32l4::Interrupt<m4::stm32l4::SPI_master>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::SPI_master> create(m4::stm32l4::SPI_master* a_p_SPI)
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::SPI_master>(a_p_SPI, IRQn_Type::SPI1_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::Polling<m4::stm32l4::SPI_master>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::SPI_master> create(m4::stm32l4::SPI_master* a_p_SPI)
    {
        return m4::stm32l4::Polling<m4::stm32l4::SPI_master>(a_p_SPI);
    }
};

template<> class Peripheral<m4::stm32l4::SPI_slave, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::SPI_slave create()
    {
        return m4::stm32l4::SPI_slave(0, SPI1);
    }
};
template<> class Peripheral<m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave> create(m4::stm32l4::SPI_slave* a_p_SPI)
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave>(a_p_SPI, IRQn_Type::SPI1_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::Polling<m4::stm32l4::SPI_slave>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::SPI_slave> create(m4::stm32l4::SPI_slave* a_p_SPI)
    {
        return m4::stm32l4::Polling<m4::stm32l4::SPI_slave>(a_p_SPI);
    }
};
#endif

#if defined(SOC_SPI2_PRESENT)
template<> class Peripheral<m4::stm32l4::SPI_master, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::SPI_master create()
    {
        return m4::stm32l4::SPI_master(1, SPI2);
    }
};
template<> class Peripheral<m4::stm32l4::Interrupt<m4::stm32l4::SPI_master>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::SPI_master> create(m4::stm32l4::SPI_master* a_p_SPI)
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::SPI_master>(a_p_SPI, IRQn_Type::SPI2_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::Polling<m4::stm32l4::SPI_master>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::SPI_master> create(m4::stm32l4::SPI_master* a_p_SPI)
    {
        return m4::stm32l4::Polling<m4::stm32l4::SPI_master>(a_p_SPI);
    }
};

template<> class Peripheral<m4::stm32l4::SPI_slave, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::SPI_slave create()
    {
        return m4::stm32l4::SPI_slave(1, SPI2);
    }
};
template<> class Peripheral<m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave> create(m4::stm32l4::SPI_slave* a_p_SPI)
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave>(a_p_SPI, IRQn_Type::SPI2_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::Polling<m4::stm32l4::SPI_slave>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::SPI_slave> create(m4::stm32l4::SPI_slave* a_p_SPI)
    {
        return m4::stm32l4::Polling<m4::stm32l4::SPI_slave>(a_p_SPI);
    }
};
#endif

#if defined(SOC_SPI3_PRESENT)
template<> class Peripheral<m4::stm32l4::SPI_master, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::SPI_master create()
    {
        return m4::stm32l4::SPI_master(2, SPI3);
    }
};
template<> class Peripheral<m4::stm32l4::Interrupt<m4::stm32l4::SPI_master>, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::SPI_master> create(m4::stm32l4::SPI_master* a_p_SPI)
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::SPI_master>(a_p_SPI, IRQn_Type::SPI3_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::Polling<m4::stm32l4::SPI_master>, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::SPI_master> create(m4::stm32l4::SPI_master* a_p_SPI)
    {
        return m4::stm32l4::Polling<m4::stm32l4::SPI_master>(a_p_SPI);
    }
};

template<> class Peripheral<m4::stm32l4::SPI_slave, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::SPI_slave create()
    {
        return m4::stm32l4::SPI_slave(2, SPI3);
    }
};
template<> class Peripheral<m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave>, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave> create(m4::stm32l4::SPI_slave* a_p_SPI)
    {
        return m4::stm32l4::Interrupt<m4::stm32l4::SPI_slave>(a_p_SPI, IRQn_Type::SPI3_IRQn);
    }
};
template<> class Peripheral<m4::stm32l4::Polling<m4::stm32l4::SPI_slave>, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::SPI_slave> create(m4::stm32l4::SPI_slave* a_p_SPI)
    {
        return m4::stm32l4::Polling<m4::stm32l4::SPI_slave>(a_p_SPI);
    }
};
#endif
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {
#if defined(SOC_SPI1_PRESENT)
template<> void rcc<SPI, 1>::enable(bool a_enable_in_lp);
template<> void rcc<SPI, 1>::disable();
#endif

#if defined(SOC_SPI2_PRESENT)
template<> void rcc<SPI, 2u>::enable(bool a_enable_in_lp);
template<> void rcc<SPI, 2u>::disable();
#endif
#if defined(SOC_SPI3_PRESENT)
template<> void rcc<SPI, 3u>::enable(bool a_enable_in_lp);
template<> void rcc<SPI, 3u>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc
