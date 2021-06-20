#pragma once

/*
 *   Name: AES.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

// soc
#include <soc/stm32l4/rcc.hpp>

// std
#include <limits>

#if defined(STM32L422xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L462xx)

namespace soc {
namespace stm32l4 {
namespace peripherals {


#ifdef AES
#undef AES
#endif

class AES : private cml::Non_copyable
{
public:

    struct Config
    {
        enum class Chaining_mode
        {
            ecb,
            cbc,
            ctr,
            gcm,
            gmac,
            ccm
        };
    
        enum class Key_size
        {
            _128b = 0x0u,
            _256b = AES_CR_KEYSIZE_Msk
        };

        Chaining_mode chaining_mode = cml::various::get_enum_incorrect_value<Chaining_mode>();
        Key_size key_size           = cml::various::get_enum_incorrect_value<Key_size>();
    };


public:
    AES();
    ~AES();

    void enable(const Config& a_config);
    void disable();

    void encrypt();
    void decrypt();

private:
};

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {
template<> struct rcc<soc::stm32l4::peripherals::AES>
{
    static void enable(bool a_enable_in_lp);
    static void disable();
};
} // namespace stm32l4
} // namespace soc

#endif