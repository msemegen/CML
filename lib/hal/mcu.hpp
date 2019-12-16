#pragma once

/*
    Name: mcu.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cortexm
#ifdef STM32L452xx
#include <hal/stm32l452xx/mcu.hpp>
#endif // STM32L452xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using c_mcu = stm32l452xx::c_mcu;
#endif // STM32L452xx

} // namespace cml
} // namespace hal
