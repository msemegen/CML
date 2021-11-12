#pragma once

#ifdef STM32L4
#include <soc/m4/stm32l4/DMA.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
template<typename Periph_t = void> using DMA1 = soc::m4::stm32l4::DMA1<Periph_t>;
template<typename Periph_t = void> using DMA2 = soc::m4::stm32l4::DMA2<Periph_t>;
#endif
} // namespace hal
} // namespace cml