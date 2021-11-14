#pragma once

/*
 *   Name: RNG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// external
#include <stm32l4xx.h>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#ifdef RNG
#undef RNG
#endif

class RNG : private cml::Non_copyable
{
public:
    ~RNG();

    bool enable(std::uint32_t a_timeout);
    void disable();

private:
    RNG();

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<> class rcc<RNG> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Factory<m4::stm32l4::RNG> : private cml::Non_constructible
{
public:
    static m4::stm32l4::RNG create()
    {
        return m4::stm32l4::RNG();
    }
};
} // namespace soc