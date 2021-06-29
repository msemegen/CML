#pragma once

/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/peripherals/I2C.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using I2C_base   = soc::m4::stm32l4::peripherals::I2C_base;
using I2C_master = soc::m4::stm32l4::peripherals::I2C_master;
using I2C_slave  = soc::m4::stm32l4::peripherals::I2C_slave;
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml