/*
    Name: crc32.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/system/crc32.hpp>

namespace soc {
namespace stm32l011xx {
namespace system {

using namespace cml;

void crc32::enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse)
{
    set_flag(&(RCC->AHBENR), RCC_AHBENR_CRCEN);

    clear_flag(&(CRC->CR), CRC_CR_POLYSIZE);
    set_flag(&(CRC->CR), static_cast<uint32_t>(a_in_reverse));
    set_flag(&(CRC->CR), static_cast<uint32_t>(a_out_reverse));
}

void crc32::disable()
{
    set_flag(&(RCC->AHBENR), RCC_AHBENR_CRCEN);
}

} // namespace system
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx