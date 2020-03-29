#pragma once

/*
    Name: USART.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/USART.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/USART.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using USART = stm32l452xx::USART;
#endif // STM32L452xx

#ifdef STM32L011xx
using USART = stm32l011xx::USART;
#endif // STM32L011xx

} // namespace hal
} // namespace cml