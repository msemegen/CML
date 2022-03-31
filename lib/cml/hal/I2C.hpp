#pragma once

/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/I2C/bsp.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using I2C        = soc::m4::stm32l4::I2C;
using I2C_master = soc::m4::stm32l4::I2C_master;
using I2C_slave  = soc::m4::stm32l4::I2C_slave;
#endif
} // namespace hal
} // namespace cml