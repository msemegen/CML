#pragma once

/*
    Name: exti_controller.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx
#include <soc/stm32l452xx/system/exti_controller.hpp>
#endif

#ifdef STM32L011xx
#include <soc/stm32l011xx/system/exti_controller.hpp>
#endif

namespace cml {
namespace hal {
namespace system {

#ifdef STM32L452xx
using exti_controller = soc::stm32l452xx::system::exti_controller;
#endif // STM32L452xx

#ifdef  STM32L011xx
using exti_controller = soc::stm32l011xx::system::exti_controller;
#endif // STM32L011xx

} // namespace system
} // namespace hal
} // namespace cml