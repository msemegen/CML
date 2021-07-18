#pragma once

#ifdef ARM_CORTEX_M4
#include <soc/m4/nvic.hpp>
#endif

namespace cml {
namespace hal {
#ifdef ARM_CORTEX_M4
using nvic = soc::m4::nvic;
#endif
} // namespace hal
} // namespace cml
