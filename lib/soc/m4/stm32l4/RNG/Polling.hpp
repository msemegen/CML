#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/Polling.hpp>
#include <soc/m4/stm32l4/RNG/RNG.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<RNG> : private cml::Non_copyable
{
public:
    bool get_value(std::uint32_t* a_p_value, std::uint32_t a_timeout);

private:
    Polling() {}

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Factory<m4::stm32l4::Polling<m4::stm32l4::RNG>> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Polling<m4::stm32l4::RNG> create()
    {
        return m4::stm32l4::Polling<m4::stm32l4::RNG>();
    }
};
} // namespace soc