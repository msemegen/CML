/*
    Name: gpio.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/gpio.hpp>

namespace
{

using namespace cml::common;

void gpio_a_enable()
{
    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
}

void gpio_a_disable()
{
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
}

void gpio_b_enable()
{
    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
}

void gpio_b_disable()
{
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
}

void gpio_c_enable()
{
    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
}

void gpio_c_disable()
{
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
}

void gpio_d_enable()
{
    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
}

void gpio_d_disable()
{
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
}

void gpio_e_enable()
{
    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
}

void gpio_e_disable()
{
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
}

void gpio_h_enable()
{
    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
}

void gpio_h_disable()
{
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
}

struct s_ll_config_data
{
    GPIO_TypeDef* p_registers = nullptr;

    void(*p_enable)()  = nullptr;
    void(*p_disable)() = nullptr;
};

s_ll_config_data ll_configs[] =
{
    { GPIOA, gpio_a_enable, gpio_a_disable },
    { GPIOB, gpio_b_enable, gpio_b_disable },
    { GPIOC, gpio_c_enable, gpio_c_disable },
    { GPIOD, gpio_d_enable, gpio_d_disable },
    { GPIOE, gpio_e_enable, gpio_e_disable },
    { GPIOH, gpio_h_enable, gpio_h_disable },
};

} // namespace ::

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;

void c_gpio::enable()
{
    this->p_gpio = ll_configs[this->to_index(this->periph)].p_registers;
    ll_configs[this->to_index(this->periph)].p_enable();

    set_bit(&(this->flags), 31u);
}

void c_gpio::disable()
{
    ll_configs[this->to_index(this->periph)].p_disable();

    common::clear_bit(&(this->flags), 31);
}

void c_output_pin::enable(const s_config& a_config)
{
    _assert(false == this->p_port->is_pin_taken(this->pin));

    _assert(c_gpio::e_pull::unknown  != a_config.pull);
    _assert(c_gpio::e_speed::unknown != a_config.speed);
    _assert(c_gpio::e_mode::unknown  != a_config.mode);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    set_flag(&(p_port->MODER), 0x3u << (this->pin * 2), 0x1u << (this->pin * 2));

    this->set_speed(a_config.speed);
    this->set_pull(a_config.pull);
    this->set_mode(a_config.mode);

    this->p_port->take_pin(this->pin);
}

void c_output_pin::disable()
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    uint32 flag = (0x3u << (this->pin * 2));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    this->p_port->give_pin(this->pin);
}

void c_output_pin::set_level(c_gpio::e_level a_level)
{
    uint8_t mul[] = { 16, 0 };
    static_cast<GPIO_TypeDef*>((*this->p_port))->BSRR = 0x1u << (this->pin + mul[static_cast<uint32>(a_level)]);
}

void c_output_pin::toggle_level()
{
    toggle_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->ODR), this->pin);
}

void c_output_pin::set_mode(c_gpio::e_mode a_t)
{
    switch (a_t)
    {
        case c_gpio::e_mode::open_drain:
        {
            set_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        case c_gpio::e_mode::push_pull:
        {
            clear_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        default:
        {
            _assert(false);
        }
    }
}

void c_output_pin::set_pull(c_gpio::e_pull a_pull)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_pull) << (this->pin * 2));
}

void c_output_pin::set_speed(c_gpio::e_speed a_speed)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_speed) << (this->pin * 2));
}

c_gpio::e_mode c_output_pin::get_mode() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<c_gpio::e_mode>(get_flag(p_port->OTYPER, static_cast<uint32>(0x1u << this->pin) << this->pin));
}

c_gpio::e_pull c_output_pin::get_pull() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<c_gpio::e_pull>(get_flag(p_port->PUPDR, static_cast<uint32>(0x1u << this->pin) << this->pin));
}

c_gpio::e_speed c_output_pin::get_speed() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<c_gpio::e_speed>((get_flag(p_port->OSPEEDR, static_cast<uint32>(0x1u << this->pin) << this->pin)));
}

void c_input_pin::enable(const s_config& a_config)
{
    _assert(false == this->p_port->is_pin_taken(this->pin));
    _assert(c_gpio::e_pull::unknown != a_config.pull);

    clear_flag(&(static_cast<GPIO_TypeDef*>((*this->p_port))->MODER), 0x3u << (this->pin * 2));

    this->set_pull(a_config.pull);
    this->p_port->take_pin(this->pin);
}

void c_input_pin::disable()
{
    const uint32 flag = (0x3u << (this->pin * 2));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    this->p_port->give_pin(this->pin);
}

c_gpio::e_level c_input_pin::read_level() const
{
    return static_cast<c_gpio::e_level>(get_bit(static_cast<GPIO_TypeDef*>((*this->p_port))->IDR, this->pin));
}

void c_input_pin::set_pull(c_gpio::e_pull a_pull)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_pull) << (this->pin * 2));
}

c_gpio::e_pull c_input_pin::get_pull() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<c_gpio::e_pull>((get_flag(p_port->PUPDR, static_cast<uint32>(0x3u << this->pin)) << this->pin));
}

void c_alternate_function_pin::enable(const s_config& a_config)
{
    _assert(false == this->p_port->is_pin_taken(this->pin));

    _assert(0 != a_config.function);
    _assert(c_gpio::e_pull::unknown  != a_config.pull);
    _assert(c_gpio::e_speed::unknown != a_config.speed);
    _assert(c_gpio::e_mode::unknown  != a_config.mode);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    set_flag(&(p_port->MODER), 0x3u << (this->pin * 2), 0x2u << (this->pin * 2));

    this->set_speed(a_config.speed);
    this->set_pull(a_config.pull);
    this->set_mode(a_config.mode);
    this->set_function(a_config.function);

    this->p_port->take_pin(this->pin);
}

void c_alternate_function_pin::disable()
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    const uint32 flag = (0x3u << (this->pin * 2));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    this->p_port->give_pin(this->pin);
}

void c_alternate_function_pin::set_mode(c_gpio::e_mode a_mode)
{
    switch (a_mode)
    {
        case c_gpio::e_mode::open_drain:
        {
            set_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        case c_gpio::e_mode::push_pull:
        {
            clear_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        default:
        {
            _assert(false);
        }
    }
}

void c_alternate_function_pin::set_pull(c_gpio::e_pull a_pull)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_pull) << (this->pin * 2));
}

void c_alternate_function_pin::set_speed(c_gpio::e_speed a_speed)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_speed) << (this->pin * 2));
}

void c_alternate_function_pin::set_function(uint32 a_function)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    uint32 af_register_index = this->pin >> 3u;
    uint32 af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((this->pin - (af_register_index * 8u)) * 4u));
    af_register |= a_function << ((this->pin - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml

#endif // STM32L0xx