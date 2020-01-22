#pragma once

/*
    Name: MCU.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/mcu.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/mcu.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using MCU = stm32l452xx::MCU;
#endif // STM32L452xx

#ifdef STM32L011xx
using MCU = stm32l011xx::MCU;
#endif

} // namespace cml
} // namespace hal
