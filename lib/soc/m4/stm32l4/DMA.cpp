// this
#include <soc/m4/stm32l4/DMA.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
void rcc<DMA1<>>::enable(bool a_enable_in_lp) {}
void rcc<DMA1<>>::disable() {}

void rcc<DMA2<>>::enable(bool a_enable_in_lp) {}
void rcc<DMA2<>>::disable() {}

}
} // namespace m4
} // namespace soc