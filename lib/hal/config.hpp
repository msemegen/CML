#pragma once

/*
    Name: config.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/config.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/config.hpp>
#endif // STM32L452xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using config = stm32l452xx::config;
#endif // STM32L452xx

#ifdef STM32L011xx
using config = stm32l011xx::config;
#endif // STM32L452xx


} // namespace hal
} // namespace cml