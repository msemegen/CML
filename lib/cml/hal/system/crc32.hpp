#pragma once

/*
    Name: crc32.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx
#include <soc/stm32l452xx/system/crc32.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/system/crc32.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace system {

#ifdef STM32L452xx
using crc32 = soc::stm32l452xx::system::crc32;
#endif // STM32L452xx

#ifdef STM32L011xx
using crc32 = soc::stm32l011xx::system::crc32;
#endif // STM32L011xx

} // namespace system
} // namespace hal
} // namespace cml