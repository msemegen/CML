#pragma once

/*
    Name: wwdg.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/system/wwdg.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/system/wwdg.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace system {

#ifdef STM32L452xx
using wwdg = soc::stm32l452xx::system::wwdg;
#endif // STM32L452xx

#ifdef STM32L011xx
using wwdg = stm32l011xx::wwdg;
#endif // STM32L011xx

} // namespace system
} // namespace hal
} // namespace cml