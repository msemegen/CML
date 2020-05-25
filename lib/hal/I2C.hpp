#pragma once

/*
    Name: I2C.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/I2C.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/I2C.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using I2C_master = stm32l452xx::I2C_master;
using I2C_slave  = stm32l452xx::I2C_slave;
#endif // STM32L452xx

#ifdef STM32L011xx
using I2C_master = stm32l011xx::I2C_master;
using I2C_slave  = stm32l011xx::I2C_slave;
#endif // STM32L011xx

} // namespace cml
} // namespace hal