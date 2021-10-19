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

template<> void rcc<GPIO>::enable<GPIOA_BASE>(Handle<GPIOA_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOASMEN);
    }
}

template<> void rcc<GPIO>::enable<GPIOB_BASE>(Handle<GPIOB_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOBSMEN);
    }
}

template<> void rcc<GPIO>::enable<GPIOC_BASE>(Handle<GPIOC_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOCSMEN);
    }
}

template<> void rcc<GPIO>::enable<GPIOH_BASE>(Handle<GPIOH_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOHSMEN);
    }
}

template<> void rcc<GPIO>::disable<GPIOA_BASE>(Handle<GPIOA_BASE>)
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOASMEN);
}

template<> void rcc<GPIO>::disable<GPIOB_BASE>(Handle<GPIOB_BASE>)
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOBSMEN);
}

template<> void rcc<GPIO>::disable<GPIOC_BASE>(Handle<GPIOC_BASE>)
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOCSMEN);
}

template<> void rcc<GPIO>::disable<GPIOH_BASE>(Handle<GPIOH_BASE>)
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOHSMEN);
}

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<GPIO>::enable<GPIOD_BASE>(Handle<GPIOD_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIODSMEN);
    }
}

template<> void rcc<GPIO>::disable<GPIOD_BASE>(Handle<GPIOD_BASE>)
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIODSMEN);
}
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<GPIO>::enable<GPIOE_BASE>(Handle<GPIOE_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOESMEN);
    }
}

template<> void rcc<GPIO>::disable<GPIOE_BASE>(Handle<GPIOE_BASE>)
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOESMEN);
}
#endif

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4