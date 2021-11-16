#pragma once

/*
 *   Name: nvic.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef M4
#include <soc/m4/nvic.hpp>
#endif

namespace cml {
namespace hal {
#ifdef M4
using nvic = soc::m4::nvic;
#endif
} // namespace hal
} // namespace cml
