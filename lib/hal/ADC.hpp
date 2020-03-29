#pragma once

/*
    Name: ADC.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/ADC.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/ADC.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using ADC = stm32l452xx::ADC;
#endif // STM32L452xx

#ifdef STM32L011xx
using ADC = stm32l011xx::ADC;
#endif // STM32L011xx

} // namespace hal
} // namespace cml