/*
 *   Name: pwr.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/stm32l4/pwr.hpp>

namespace soc {
namespace stm32l4 {
pwr::Core_sleep_mode pwr::core_sleep_mode = pwr::Core_sleep_mode::unknown;
} // namespace stm32l4
} // namespace soc