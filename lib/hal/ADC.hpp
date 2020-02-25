#pragma once

/*
    Name: ADC.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/ADC.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using ADC = stm32l452xx::ADC;
#endif

} // namespace hal
} // namespace cml