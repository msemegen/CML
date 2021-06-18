/*
 *   Name: pwr.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/stm32l4/pwr.hpp>

// cml
#include <cml/various.hpp>

namespace soc {
namespace stm32l4 {

using namespace cml;

pwr::Core_sleep_mode pwr::core_sleep_mode = various::enum_incorrect_value<pwr::Core_sleep_mode>();
} // namespace stm32l4
} // namespace soc