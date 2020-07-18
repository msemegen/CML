/*
    Name: wwdg.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <soc/stm32l011xx/system/wwdg.hpp>

//cml
#include <cml/bit.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx::system;

uint16_t reload = 0;

wwdg::Callback callback;

} // namespace ::

extern "C"
{

void WWDG_IRQHandler()
{
    if (nullptr != callback.function)
    {
        callback.function(callback.p_user_data);
    }
}

} // extern C

namespace soc {
namespace stm32l011xx {
namespace system {

using namespace cml;

void wwdg::enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window, uint16_t a_irq_priority)
{
    WWDG->CR = (WWDG_CR_WDGA | a_reload);
    WWDG->CFR = static_cast<uint32_t>(a_prescaler) | a_window;

    NVIC_SetPriority(WWDG_IRQn, a_irq_priority);
    NVIC_EnableIRQ(WWDG_IRQn);
}

void wwdg::register_early_wakeup_callback(const Callback& a_callback)
{
    callback = a_callback;
    set_flag(&(WWDG->CFR), WWDG_CFR_EWI);
}

void wwdg::unregister_early_wakeup_callback()
{
    clear_flag(&(WWDG->CFR), WWDG_CFR_EWI);
    callback = { nullptr, nullptr };
}

void wwdg::feed()
{
    WWDG->CR = reload;
}

} // namespace system
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx