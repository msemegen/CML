#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Polling.hpp>
#include <soc/m4/stm32l4/RNG/RNG.hpp>

// cml
#include <cml/Non_constructible.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<RNG> : private cml::Non_constructible
{
public:
    static bool get_value(std::uint32_t* a_p_value, std::uint32_t a_timeout);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc