#pragma once

/*
    Name: usart.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class c_usart
{
public:

    void write_bytes_polling(const void* a_p_data, common::uint32 a_data_length);
    bool write_bytes_polling(const void* a_p_data, common::uint32 a_data_length, common::time_tick a_timeout);

    void read_bytes_polling(void* a_p_data, common::uint32 a_data_length);
    bool read_bytes_polling(void* a_p_data, common::uint32 a_data_length, common::time_tick a_timeout);
};

} // namespace stm32l011xx
} // namespace hal
} // namespace cml