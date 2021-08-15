#pragma once

/*
 *   Name: TIM.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/peripherals/TIM.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
class TIM : private Non_constructible
{
public:
    using Basic = soc::m4::stm32l4::peripherals::TIM::Basic;
};
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml