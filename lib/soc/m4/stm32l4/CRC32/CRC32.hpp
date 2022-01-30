#pragma once

/*
 *   Name: CRC32.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class CRC32 : private cml::Non_copyable
{
public:
    enum class In_data_reverse : std::uint32_t
    {
        none      = 0,
        byte      = CRC_CR_REV_IN_0,
        half_word = CRC_CR_REV_IN_1,
        word      = CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1
    };

    enum class Out_data_reverse : std::uint32_t
    {
        none    = 0,
        enabled = CRC_CR_REV_OUT
    };

    CRC32(CRC32&&) = default;
    CRC32& operator=(CRC32&&) = default;

    CRC32()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , enabled(false)
    {
    }
    ~CRC32()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse);
    void disable();

    std::uint32_t calculate(const std::uint8_t* a_p_data, std::uint32_t a_data_size);

    std::uint32_t get_idx()
    {
        return this->idx;
    }

    bool is_enabled() const
    {
        return this->enabled;
    }

    bool is_created() const
    {
        return this->idx != std::numeric_limits<decltype(this->idx)>::max();
    }

    operator CRC_TypeDef*()
    {
        return CRC;
    }

    operator const CRC_TypeDef*() const
    {
        return CRC;
    }

private:
    CRC32(std::uint32_t a_idx)
        : idx(a_idx)
        , enabled(false)
    {
    }

    std::uint32_t idx;
    bool enabled;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

template<> class rcc<CRC32> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();
};
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Peripheral<m4::stm32l4::CRC32> : private cml::Non_constructible
{
public:
    static m4::stm32l4::CRC32 create()
    {
        return m4::stm32l4::CRC32(0);
    }
};
} // namespace soc
