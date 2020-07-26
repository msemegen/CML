#pragma once

/*
    Name: GPIO.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/GPIO.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/GPIO.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using GPIO = soc::stm32l452xx::peripherals::GPIO;
using pin  = soc::stm32l452xx::peripherals::pin;
#endif // STM32L452xx

#ifdef STM32L011xx
using GPIO                   = soc::stm32l011xx::peripherals::GPIO;
using Output_pin             = soc::stm32l011xx::peripherals::Output_pin;
using Input_pin              = soc::stm32l011xx::peripherals::Input_pin;
using Alternate_function_pin = soc::stm32l011xx::peripherals::Alternate_function_pin;
using Analog_pin             = soc::stm32l011xx::peripherals::Analog_pin;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml