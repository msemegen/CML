#pragma once

/*
    Name: gpio.hpp

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
using c_gpio                   = stm32l452xx::c_gpio;
using c_output_pin             = stm32l452xx::c_output_pin;
using c_input_pin              = stm32l452xx::c_input_pin;
using c_alternate_function_pin = stm32l452xx::c_alternate_function_pin;
using c_analog_pin             = stm32l452xx::c_analog_pin;
#endif // STM32L452xx

#ifdef STM32L011xx
using c_gpio                   = stm32l011xx::c_gpio;
using c_output_pin             = stm32l011xx::c_output_pin;
using c_input_pin              = stm32l011xx::c_input_pin;
using c_alternate_function_pin = stm32l011xx::c_alternate_function_pin;
using c_analog_pin             = stm32l011xx::c_analog_pin;
#endif

} // namespace hal
} // namespace cml