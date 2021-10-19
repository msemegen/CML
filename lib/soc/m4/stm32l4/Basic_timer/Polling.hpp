#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>
#include <soc/m4/stm32l4/Polling.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<Basic_timer> : private cml::Non_copyable
{
public:
    Polling(Basic_timer* a_p_timer)
        : p_timer(a_p_timer)
    {
    }

    bool is_overload() const
    {
        return cml::bit::is(static_cast<TIM_TypeDef*>(*(this->p_timer))->CNT, 31u);
    }

private:
    Basic_timer* p_timer;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc