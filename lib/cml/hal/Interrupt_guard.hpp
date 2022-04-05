#pragma once

/*
 *   Name: Interrupt_guard.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/Interrupt_guard.hpp>

namespace cml {
namespace hal {
using Interrupt_guard = soc::m4::Interrupt_guard;
} // namespace hal
} // namespace cml