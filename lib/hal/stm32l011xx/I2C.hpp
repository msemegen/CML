#pragma once

//externals
#include <stm32l011xx.h>

//cml
#include <common/Non_copyable.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class I2C_master : public common::Non_copyable
{

};

class I2C_slave : public common::Non_copyable
{

};

} // namespace stm32l011xx
} // namespace hal
} // namespace cml