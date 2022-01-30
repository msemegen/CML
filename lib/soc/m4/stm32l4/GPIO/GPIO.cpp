/*
 *   Name: GPIO.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace {
using namespace soc::m4::stm32l4;

GPIO::Interrupt::Callback callbacks[7u];
} // namespace

extern "C" {
using namespace cml;

static bool interrupt_handler(std::uint32_t a_pr1, std::uint32_t a_bit_index, std::uint32_t a_handler_index)
{
    if (true == bit::is(a_pr1, a_bit_index))
    {
        callbacks[a_handler_index].function(a_bit_index, callbacks[a_handler_index].p_user_data);
        return true;
    }

    return false;
}

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 0u, 0u))
    {
        bit::set(&(EXTI->PR1), 0u);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 1u, 1u))
    {
        bit::set(&(EXTI->PR1), 1u);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 2u, 2u))
    {
        bit::set(&(EXTI->PR1), 2u);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 3u, 3u))
    {
        bit::set(&(EXTI->PR1), 3u);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 4u, 4u))
    {
        bit::set(&(EXTI->PR1), 4u);
    }
}

void EXTI9_5_IRQHandler()
{
    for (std::uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i, 5u))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (std::uint32_t i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i, 6u))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void GPIO::In::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2));
}

GPIO::Level GPIO::In::Pin::get_level() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Level>(bit::is(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

GPIO::Pull GPIO::In::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(
        (bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR, static_cast<std::uint32_t>(0x3u << this->id))
         << this->id));
}

void GPIO::Out::Pin::set_level(Level a_level)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    constexpr std::uint8_t mul[]                      = { 16u, 0u };
    static_cast<GPIO_TypeDef*>(*(this->p_port))->BSRR = 0x1u << (this->id + mul[static_cast<std::uint32_t>(a_level)]);
}

void GPIO::Out::Pin::toggle_level()
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::toggle(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->ODR), this->id);
}

void GPIO::Out::Pin::set_mode(Mode a_mode)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<std::uint32_t>(a_mode) << this->id);
}

void GPIO::Out::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2));
}

void GPIO::Out::Pin::set_speed(Speed a_speed)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
                  0x3u << (this->id * 2),
                  static_cast<std::uint32_t>(a_speed) << (this->id * 2u));
}

GPIO::Level GPIO::Out::Pin::get_level() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Level>(bit::is(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

GPIO::Mode GPIO::Out::Pin::get_mode() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Mode>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Pull GPIO::Out::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Speed GPIO::Out::Pin::get_speed() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Speed>((bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                             static_cast<std::uint32_t>(0x1u << this->id) << this->id)));
}

void GPIO::Analog::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2u),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2));
}

GPIO::Pull GPIO::Analog::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

void GPIO::Alternate_function::Pin::set_mode(Mode a_mode)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<std::uint32_t>(a_mode) << this->id);
}

void GPIO::Alternate_function::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2u),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2u));
}

void GPIO::Alternate_function::Pin::set_speed(Speed a_speed)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
                  0x3u << (this->id * 2u),
                  static_cast<std::uint32_t>(a_speed) << (this->id * 2u));
}

void GPIO::Alternate_function::Pin::set_function(std::uint32_t a_function)
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    std::uint32_t af_register_index = this->id >> 3u;
    std::uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((this->id - (af_register_index * 8u)) * 4u));
    af_register |= a_function << ((this->id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;
    this->function                 = a_function;
}

GPIO::Mode GPIO::Alternate_function::Pin::get_mode() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Mode>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Pull GPIO::Alternate_function::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Speed GPIO::Alternate_function::Pin::get_speed() const
{
    cml_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Speed>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                            static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

void GPIO::In::enable(std::uint32_t a_id, Pull a_pull, Pin* a_p_pin)
{
    cml_assert(a_id < 16u);

    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*(this->p_port)));

    bit_flag::set(&(p_port->PUPDR), 0x3u << (a_id * 2u), static_cast<std::uint32_t>(a_pull) << (a_id * 2u));
    bit_flag::clear(&(p_port->MODER), 0x3u << (a_id * 2u));

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id     = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::In::disable(std::uint32_t a_id)
{
    cml_assert(a_id < 16u);

    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const std::uint32_t flag = (0x3u << (a_id * 2u));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}

void GPIO::Out::enable(std::uint32_t a_id, const Enable_config& a_config, Pin* a_p_pin)
{
    cml_assert(a_id < 16u);

    cml_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    cml_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    cml_assert(various::get_enum_incorrect_value<Mode>() != a_config.mode);

    cml_assert(false == this->p_port->is_pin_taken(a_id));

    const std::uint32_t clear_flag_2bit = 0x3u << (a_id * 2);
    GPIO_TypeDef* p_port                = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.speed) << (a_id * 2u));
    bit_flag::set(&(p_port->PUPDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.pull) << (a_id * 2u));
    bit_flag::set(&(p_port->MODER), clear_flag_2bit, 0x1u << (a_id * 2u));
    bit_flag::set(&(p_port->OTYPER), 0x1u << a_id, static_cast<std::uint32_t>(a_config.mode) << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id     = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::Out::disable(std::uint32_t a_id)
{
    cml_assert(a_id < 16u);

    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const std::uint32_t flag = (0x3u << (a_id * 2u));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->OSPEEDR), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}

void GPIO::Analog::enable(std::uint32_t a_id, Pull a_pull, Pin* a_p_out_pin)
{
    cml_assert(a_id < 16u);
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->PUPDR), 0x3u << (a_id * 2u), static_cast<std::uint32_t>(a_pull) << (a_id * 2u));
    bit_flag::set(&(p_port->MODER), 0x3u << (a_id * 2u), 0x3u << (a_id * 2u));

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id     = a_id;
        a_p_out_pin->p_port = this->p_port;
    }
}

void GPIO::Analog::disable(std::uint32_t a_id)
{
    cml_assert(a_id < 16u);

    bit_flag::clear(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR), (0x3u << (a_id * 2u)));

    this->p_port->give_pin(a_id);
}

void GPIO::Alternate_function::enable(std::uint32_t a_id, const Enable_config& a_config, Pin* a_p_pin)
{
    cml_assert(a_id < 16u);

    cml_assert(false == this->p_port->is_pin_taken(a_id));

    cml_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    cml_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    cml_assert(various::get_enum_incorrect_value<Mode>() != a_config.mode);

    const std::uint32_t clear_flag_2bit = 0x3u << (a_id * 2);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.speed) << (a_id * 2u));
    bit_flag::set(&(p_port->PUPDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.pull) << (a_id * 2u));
    bit_flag::set(&(p_port->MODER), clear_flag_2bit, 0x2u << (a_id * 2u));
    bit_flag::set(&(p_port->OTYPER), 0x1u << a_id, static_cast<std::uint32_t>(a_config.mode) << a_id);

    std::uint32_t af_register_index = a_id >> 3u;
    std::uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((a_id - (af_register_index * 8u)) * 4u));
    af_register |= a_config.function << ((a_id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id       = a_id;
        a_p_pin->p_port   = this->p_port;
        a_p_pin->function = a_config.function;
    }
}

void GPIO::Alternate_function::disable(std::uint32_t a_id)
{
    cml_assert(a_id < 16u);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const std::uint32_t flag = (0x3u << (a_id * 2u));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->OSPEEDR), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}

void GPIO::Interrupt::enable(const Callback& a_callback, const IRQ_config& a_irq_config)
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);
    cml_assert(true == rcc<mcu>::is_SYSCFG_active());

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    callbacks[this->idx] = a_callback;
}

void GPIO::Interrupt::disable()
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    NVIC_DisableIRQ(this->irqn);

    callbacks[this->idx] = { nullptr, nullptr };
}

void GPIO::Interrupt::attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Mode a_mode)
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    volatile std::uint32_t* p_register = &(SYSCFG->EXTICR[a_pin / 4u]);
    std::uint32_t pos                  = ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u);

#ifdef CML_ASSERT_ENABLED
    const bool f = bit_flag::is(*p_register, (a_port.get_id()) << pos);
    cml_assert((0u == a_port.get_id() && true == f) || (0u != a_port.get_id() && false == f));
    cml_assert((0u == this->idx && 0u == a_pin) || (1u == this->idx && 1u == a_pin) ||
               (2u == this->idx && 2u == a_pin) || (3u == this->idx && 3u == a_pin) ||
               (4u == this->idx && 4u == a_pin) || (5u == this->idx && (a_pin >= 5u && a_pin <= 9u)) ||
               (6u == this->idx && (a_pin >= 10u && a_pin <= 15u)));
#endif

    Interrupt_guard guard;

    bit_flag::set(p_register, 0x3u << pos, a_port.get_id() << pos);

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    switch (a_mode)
    {
        case Mode::event: {
            bit::set(&(EXTI->EMR1), a_pin);
        }
        break;

        case Mode::interrupt: {
            bit::set(&(EXTI->IMR1), a_pin);
        }
        break;
    }

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI->RTSR1), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI->FTSR1), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI->RTSR1), a_pin);
                bit::set(&(EXTI->FTSR1), a_pin);
            }
        }
    }
}

void GPIO::Interrupt::deattach(const GPIO& a_port, std::uint32_t a_pin)
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    Interrupt_guard guard;

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    bit::clear(&(EXTI->EMR1), a_pin);
    bit::clear(&(EXTI->IMR1), a_pin);

    bit_flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]), a_port.get_id() << ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u));

    callbacks[this->idx] = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4