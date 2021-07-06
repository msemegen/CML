#pragma once

/*
 *   Name: rcc.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

namespace soc {
namespace m4 {
namespace stm32l4 {

template<typename Periph_t> class rcc
{
private:
    rcc()           = delete;
    rcc(const rcc&) = delete;
    rcc(rcc&&)      = delete;
    ~rcc()          = delete;

    rcc& operator=(const rcc&) = delete;
    rcc& operator=(rcc&&) = delete;
};

} // namespace stm32l4
} // namespace m4
} // namespace soc