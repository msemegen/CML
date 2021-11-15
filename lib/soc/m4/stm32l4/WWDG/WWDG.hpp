#pragma once

/*
    Name: WWDG.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>
#include <limits>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#ifdef WWDG
#undef WWDG
#endif

class WWDG : private cml::Non_copyable
{
public:
    enum class Prescaler : uint32_t
    {
        _1 = 0,
        _2 = WWDG_CFR_WDGTB_0,
        _4 = WWDG_CFR_WDGTB_1,
        _8 = WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1
    };

    WWDG()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
    }

    void enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window);
    void feed();

    std::uint32_t get_idx()
    {
        return this->idx;
    }

    operator WWDG_TypeDef*()
    {
        return reinterpret_cast<WWDG_TypeDef*>(WWDG_BASE);
    }

    operator const WWDG_TypeDef*() const
    {
        return reinterpret_cast<WWDG_TypeDef*>(WWDG_BASE);
    }

private:
    WWDG(std::uint32_t a_idx)
        : idx(a_idx)
    {
    }
    std::uint32_t idx;

    std::uint16_t reload;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
template<> class Factory<m4::stm32l4::WWDG> : private cml::Non_constructible
{
public:
    static m4::stm32l4::WWDG create()
    {
        return m4::stm32l4::WWDG(0);
    }
};
} // namespace soc