#pragma once

/*
    Name: usart.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/usart.hpp>
#endif

#ifdef STM32L011xx
#include <hal/stm32l011xx/usart.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using USART = stm32l452xx::USART;
#endif

#ifdef STM32L011xx
using USART = stm32l011xx::USART;
#endif

} // namespace hal
} // namespace cml