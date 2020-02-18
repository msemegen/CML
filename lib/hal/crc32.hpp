#pragma once

/*
    Name: crc32.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx
#include <hal/stm32l452xx/crc32.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/crc32.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using crc32 = stm32l452xx::crc32;
#endif // STM32L452xx

#ifdef STM32L011xx
using crc32 = stm32l011xx::crc32;
#endif

} // namespace cml
} // namespace hal