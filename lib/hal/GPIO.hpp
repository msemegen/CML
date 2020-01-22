#pragma once

/*
    Name: GPIO.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx
#include <hal/stm32l452xx/gpio.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/gpio.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using GPIO                   = stm32l452xx::GPIO;
using Output_pin             = stm32l452xx::Output_pin;
using Input_pin              = stm32l452xx::Input_pin;
using Alternate_function_pin = stm32l452xx::Alternate_function_pin;
#endif // STM32L452xx

#ifdef STM32L011xx
using GPIO                   = stm32l011xx::GPIO;
using Output_pin             = stm32l011xx::Output_pin;
using Input_pin              = stm32l011xx::Input_pin;
using Alternate_function_pin = stm32l011xx::Alternate_function_pin;
#endif

} // namespace hal
} // namespace cml