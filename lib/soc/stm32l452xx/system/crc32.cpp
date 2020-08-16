/*
    Name: crc32.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/system/crc32.hpp>

namespace soc {
namespace stm32l452xx {
namespace system {

using namespace cml;

void crc32::enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse)
{
    set_flag(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);

    clear_flag(&(CRC->CR), CRC_CR_POLYSIZE);
    set_flag(&(CRC->CR), static_cast<uint32_t>(a_in_reverse));
    set_flag(&(CRC->CR), static_cast<uint32_t>(a_out_reverse));
}

void crc32::disable()
{
    clear_flag(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);
}

} // namespace system
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx