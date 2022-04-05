#pragma once

/*
 *   Name: cycles_counter.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef M4
#include <soc/m4/cycles_counter.hpp>
#endif

namespace cml {
namespace hal {
#ifdef M4
using cycles_counter = soc::m4::cycles_counter;
#endif
} // namespace hal
} // namespace cml