#pragma once

/*
 *   Name: EXTI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/Non_copyable.hpp>

// external
#ifdef STM32L4
#include <stm32l4xx.h>
#endif

namespace soc {
namespace m4 {
namespace stm32l4 {

#ifdef EXTI
#undef EXTI
#endif

template<typename Peripheral_t> class EXTI : private cml::Non_copyable
{
};

} // namespace stm32l4
} // namespace m4
} // namespace soc