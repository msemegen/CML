#pragma once

/*
 *   Name: IRQ_config.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#ifdef M4
#include <soc/m4/IRQ_config.hpp>
#endif

namespace cml {
namespace hal {
#ifdef M4
using IRQ_config = soc::m4::IRQ_config;
#endif
} // namespace hal
} // namespace cml