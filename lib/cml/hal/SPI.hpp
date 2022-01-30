#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/SPI/bsp/bsp.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using SPI        = soc::m4::stm32l4::SPI;
using SPI_master = soc::m4::stm32l4::SPI_master;
using SPI_slave  = soc::m4::stm32l4::SPI_slave;
#endif
} // namespace hal
} // namespace cml