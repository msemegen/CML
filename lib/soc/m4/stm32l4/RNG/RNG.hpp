#pragma once

/*
 *   Name: RNG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

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
    RNG()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
    }
    ~RNG();

    bool enable(std::uint32_t a_timeout);
    void disable();

    std::uint32_t get_idx()
    {
        return this->idx;
    }

    operator RNG_TypeDef*()
    {
        return reinterpret_cast<RNG_TypeDef*>(RNG_BASE);
    }

    operator const RNG_TypeDef*() const
    {
        return reinterpret_cast<RNG_TypeDef*>(RNG_BASE);
    }

private:
    RNG(std::uint32_t a_idx)
        : idx(a_idx)
    {
    }

    std::uint32_t idx;
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
        return m4::stm32l4::RNG(0);
    }
};
} // namespace soc