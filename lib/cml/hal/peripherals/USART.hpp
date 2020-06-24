#pragma once

/*
    Name: USART.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/USART.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/USART.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using USART = soc::stm32l452xx::peripherals::USART;
#endif // STM32L452xx

#ifdef STM32L011xx
using USART = soc::stm32l011xx::peripherals::USART;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml