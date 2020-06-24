#pragma once

/*
    Name: rng.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/system/rng.hpp>
#endif // STM32L452xx

namespace cml {
namespace hal {
namespace system {

#ifdef STM32L452xx
using rng = soc::stm32l452xx::system::rng;
#endif // STM32L452xx

} // namespace system
} // namespace hal
} // namespace cml