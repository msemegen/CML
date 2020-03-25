#pragma once

/*
    Name: rng.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/rng.hpp>
#endif // STM32L452xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using rng = stm32l452xx::rng;
#endif // STM32L452xx

} // namespace hal
} // namespace cml