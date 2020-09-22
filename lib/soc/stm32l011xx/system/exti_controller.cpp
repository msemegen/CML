/*
    Name: exti_controller.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/system/exti_controller.hpp>

// soc
#ifdef CML_ASSERT
#include <soc/stm32l011xx/mcu.hpp>
#endif
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/debug/assert.hpp>

namespace {

using namespace soc::stm32l011xx::peripherals;
using namespace soc::stm32l011xx::system;

struct Handler
{
    exti_controller::Callback callback;
    pin::In const* p_pin = nullptr;
};

Handler handlers[16];

} // namespace

extern "C" {

using namespace cml;

static void interrupt_handler(uint32_t a_index)
{
    assert(nullptr != handlers[a_index].callback.function);

    if (true == is_bit(EXTI->PR, a_index))
    {
        handlers[a_index].callback.function(handlers[a_index].p_pin->get_level(),
                                            handlers[a_index].callback.p_user_data);

        set_bit(&(EXTI->PR), a_index);
    }
}

void EXTI0_1_IRQHandler()
{
    interrupt_handler(0);
    interrupt_handler(1);
}

void EXTI2_3_IRQHandler()
{
    interrupt_handler(2);
    interrupt_handler(3);
}

void EXTI4_15_IRQHandler()
{
    for (uint32_t i = 4u; i <= 15u; i++)
    {
        interrupt_handler(i);
    }
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace system {

using namespace cml;

void exti_controller::enable(uint32_t a_priority)
{
    NVIC_SetPriority(EXTI0_1_IRQn, a_priority);
    NVIC_SetPriority(EXTI2_3_IRQn, a_priority);
    NVIC_SetPriority(EXTI4_15_IRQn, a_priority);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void exti_controller::disable()
{
    NVIC_DisableIRQ(EXTI0_1_IRQn);
    NVIC_DisableIRQ(EXTI2_3_IRQn);
    NVIC_DisableIRQ(EXTI4_15_IRQn);
}

void exti_controller::register_callback(pin::In* a_p_pin, Interrupt_mode a_mode, const Callback& a_callback)
{
    assert(nullptr != a_p_pin);
    assert(true == mcu::is_syscfg_enabled());
    assert(nullptr == handlers[static_cast<uint32_t>(a_p_pin->get_id())].callback.function);

    Interrupt_guard guard;

    set_flag(&(SYSCFG->EXTICR[a_p_pin->get_id() / 4u]),
             (static_cast<uint32_t>(a_p_pin->get_port()->get_id())
              << ((static_cast<uint32_t>(a_p_pin->get_id()) % 4u) * 4u)));

    clear_bit(&(EXTI->RTSR), a_p_pin->get_id());
    clear_bit(&(EXTI->FTSR), a_p_pin->get_id());
    set_bit(&(EXTI->IMR), a_p_pin->get_id());

    switch (a_mode)
    {
        case Interrupt_mode::rising: {
            set_bit(&(EXTI->RTSR), a_p_pin->get_id());
        }
        break;

        case Interrupt_mode::falling: {
            set_bit(&(EXTI->FTSR), a_p_pin->get_id());
        }
        break;

        default: {
            if ((Interrupt_mode::rising | Interrupt_mode::falling) == a_mode)
            {
                set_bit(&(EXTI->RTSR), a_p_pin->get_id());
                set_bit(&(EXTI->FTSR), a_p_pin->get_id());
            }
        }
    }

    handlers[a_p_pin->get_id()] = { a_callback, a_p_pin };
}

void exti_controller::unregister_callback(const pin::In& a_pin)
{
    Interrupt_guard guard;

    clear_bit(&(EXTI->RTSR), a_pin.get_id());
    clear_bit(&(EXTI->FTSR), a_pin.get_id());

    clear_flag(
        &(SYSCFG->EXTICR[a_pin.get_id() / 4u]),
        (static_cast<uint32_t>(a_pin.get_port()->get_id()) << ((static_cast<uint32_t>(a_pin.get_id()) % 4u) * 4u)));

    handlers[a_pin.get_id()] = { { nullptr, nullptr }, nullptr };
}

} // namespace system
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx