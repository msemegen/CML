#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/SPI.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/SPI.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using SPI_base   = soc::stm32l452xx::peripherals::SPI_base;
using SPI_master = soc::stm32l452xx::peripherals::SPI_master;
using SPI_slave  = soc::stm32l452xx::peripherals::SPI_slave;
#endif

#ifdef STM32L011xx
using SPI_base   = soc::stm32l011xx::peripherals::SPI_base;
using SPI_master = soc::stm32l011xx::peripherals::SPI_master;
using SPI_slave  = soc::stm32l011xx::peripherals::SPI_slave;
#endif

} // namespace peripherals
} // namespace hal
} // namespace cml