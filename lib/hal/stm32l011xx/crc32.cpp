/*
    Name: crc32.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <hal/stm32l011xx/crc32.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;

void crc32::enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse)
{
    set_flag(&(RCC->AHBENR), RCC_AHBENR_CRCEN);

    clear_flag(&(CRC->CR), CRC_CR_POLYSIZE);
    set_flag(&(CRC->CR), static_cast<uint32>(a_in_reverse));
    set_flag(&(CRC->CR), static_cast<uint32>(a_out_reverse));
}

void crc32::disable()
{
    set_flag(&(RCC->AHBENR), RCC_AHBENR_CRCEN);
}

} // namespace cml
} // namespace hal
} // namespace stm32l011xx

#endif // STM32L011xx