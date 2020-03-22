#pragma once

/*
    Name: wwdg.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifdef STM32L452xx
#include <hal/stm32l452xx/wwdg.hpp>
#endif

#ifdef STM32L011xx
#include <hal/stm32l011xx/wwdg.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using wwdg = stm32l452xx::wwdg;
#endif

#ifdef STM32L011xx
using wwdg = stm32l011xx::wwdg;
#endif

} // namespace hal
} // namespace cml