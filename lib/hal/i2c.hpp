#pragma once

#ifdef STM32L452xx
#include <hal/stm32l452xx/i2c_master.hpp>
#include <hal/stm32l452xx/i2c_slave.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using c_i2c_master = stm32l4xx::c_i2c_master;
using c_i2c_slave  = stm32l4xx::c_i2c_slave;
#endif

} // namespace hal
} // namespace cml