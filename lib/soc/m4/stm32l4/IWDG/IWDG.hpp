#pragma once

/*
 *   Name: IWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/Factory.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#ifdef IWDG
#undef IWDG
#endif
class IWDG : private cml::Non_copyable
{
public:
    enum class Prescaler : std::uint32_t
    {
        _4   = 0x0u,
        _8   = IWDG_PR_PR_0,
        _16  = IWDG_PR_PR_1,
        _32  = IWDG_PR_PR_1 | IWDG_PR_PR_0,
        _64  = IWDG_PR_PR_2,
        _128 = IWDG_PR_PR_2 | IWDG_PR_PR_0,
        _256 = IWDG_PR_PR_2 | IWDG_PR_PR_1
    };

    struct Window
    {
        enum class Mode : std::uint32_t
        {
            disabled,
            enabled
        };

        Mode mode           = cml::various::get_enum_incorrect_value<Mode>();
        std::uint16_t value = 0xFFFu;
    };

    ~IWDG();

    bool enable(Prescaler a_prescaler, std::uint16_t a_reload, const Window& a_window, std::uint32_t a_timeout);
    void feed();

private:
    IWDG();

    template<typename Periph_t, std::size_t id> friend class Factory;
};

template<> class Factory<IWDG> : private cml::Non_constructible
{
public:
    static IWDG create()
    {
        return IWDG();
    }
};
} // namespace stm32l4
} // namespace m4
} // namespace soc