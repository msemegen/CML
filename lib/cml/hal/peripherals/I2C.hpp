#pragma once

/*
    Name: I2C.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/I2C.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/I2C.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using I2C_base   = soc::stm32l452xx::peripherals::I2C_base;
using I2C_master = soc::stm32l452xx::peripherals::I2C_master;
using I2C_slave  = soc::stm32l452xx::peripherals::I2C_slave;
#endif // STM32L452xx

#ifdef STM32L011xx
using I2C_base   = soc::stm32l011xx::peripherals::I2C_base;
using I2C_master = soc::stm32l011xx::peripherals::I2C_master;
using I2C_slave  = soc::stm32l011xx::peripherals::I2C_slave;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml