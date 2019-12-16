#pragma once

/*
    Name: config.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cortexm
#ifdef STM32L452xx
#include <hal/st/stm32l452xx/config.hpp>
#endif // STM32L452xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using s_config = st::stm32l452xx::s_config;
#endif // STM32L452xx

} // namespace hal
} // namespace cml