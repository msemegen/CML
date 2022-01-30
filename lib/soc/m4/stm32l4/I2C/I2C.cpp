/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/I2C/I2C.hpp>

// soc
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void I2C::disable()
{
    this->p_registers->CR1 = 0;

    if (true == bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx))
    {
        bit::clear(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx);
    }
}

void I2C_master::enable(const Enable_config& a_config)
{
    cml_assert(true == this->is_created());

    cml_assert(various::get_enum_incorrect_value<Enable_config::Analog_filter>() != a_config.analog_filter);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Fast_plus>() != a_config.fast_plus);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Crc>() != a_config.crc);

    cml_assert((Enable_config::Fast_plus::enabled == a_config.fast_plus && true == rcc<mcu>::is_SYSCFG_active()) ||
               Enable_config::Fast_plus::disabled == a_config.fast_plus);

    this->p_registers->CR1 = 0;

    this->p_registers->TIMINGR = a_config.timings;
    this->p_registers->CR2     = I2C_CR2_AUTOEND;
    this->p_registers->CR1 = (Enable_config::Analog_filter::disabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                             (Enable_config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (Enable_config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx);
    }
}

I2C_master::Enable_config I2C_master::get_Enable_config() const
{
    return { static_cast<Enable_config::Analog_filter>(false == bit_flag::is(this->p_registers->CR1, I2C_CR1_ANFOFF)),
             static_cast<Enable_config::Fast_plus>(bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx)),
             static_cast<Enable_config::Crc>(bit_flag::is(this->p_registers->CR1, I2C_CR1_PECEN)),
             this->p_registers->TIMINGR };
}

void I2C_slave::enable(const Enable_config& a_config)
{
    cml_assert(true == this->is_created());

    cml_assert(various::get_enum_incorrect_value<Enable_config::Analog_filter>() != a_config.analog_filter);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Fast_plus>() != a_config.fast_plus);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Crc>() != a_config.crc);
    cml_assert(a_config.address <= 0x7Fu);

    cml_assert((Enable_config::Fast_plus::enabled == a_config.fast_plus && true == rcc<mcu>::is_SYSCFG_active()) ||
               Enable_config::Fast_plus::disabled == a_config.fast_plus);

    this->p_registers->CR1     = 0;
    this->p_registers->TIMINGR = a_config.timings;
    this->p_registers->OAR1    = I2C_OAR1_OA1EN | (a_config.address);
    this->p_registers->CR1 = (Enable_config::Analog_filter::enabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0u) |
                             (Enable_config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0u) | I2C_CR1_PE;

    if (Enable_config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx);
    }
}

I2C_slave::Enable_config I2C_slave::get_Enable_config() const
{
    return { static_cast<Enable_config::Analog_filter>(false == bit_flag::is(this->p_registers->CR1, I2C_CR1_ANFOFF)),
             static_cast<Enable_config::Fast_plus>(bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx)),
             static_cast<Enable_config::Crc>(bit_flag::is(this->p_registers->CR1, I2C_CR1_PECEN)),
             this->p_registers->TIMINGR,
             static_cast<std::uint16_t>(this->p_registers->OAR1 & 0x8000u) };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif