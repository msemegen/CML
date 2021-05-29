#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/peripherals/SPI.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using SPI_base   = soc::stm32l4::peripherals::SPI_base;
using SPI_master = soc::stm32l4::peripherals::SPI_master;
using SPI_slave  = soc::stm32l4::peripherals::SPI_slave;
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml