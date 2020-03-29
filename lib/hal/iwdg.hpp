#pragma once

/*
    Name: iwdg.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/iwdg.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/iwdg.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using iwdg = stm32l452xx::iwdg;
#endif // STM32L452xx

#ifdef STM32L011xx
using iwdg = stm32l011xx::iwdg;
#endif // STM32L011xx

} // namespace hal
} // namespace cml