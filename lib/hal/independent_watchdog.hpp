#pragma once

/*
    Name: independent_watchdog.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/independent_watchdog.hpp>
#endif

#ifdef STM32L011xx
#include <hal/stm32l011xx/independent_watchdog.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using independent_watchdog = stm32l452xx::independent_watchdog;
#endif

#ifdef STM32L011xx
using independent_watchdog = stm32l011xx::independent_watchdog;
#endif

} // namespace hal
} // namespace cml