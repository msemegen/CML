#pragma once

/*
    Name: mcu.hpp

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
using mcu = stm32l452xx::mcu;
#endif // STM32L452xx

#ifdef STM32L011xx
using mcu = stm32l011xx::mcu;
#endif // STM32L011xx

} // namespace cml
} // namespace hal
