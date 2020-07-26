/*
    Name: GPIO.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <soc/stm32l452xx/peripherals/GPIO.hpp>

namespace
{

using namespace cml;
using namespace soc::stm32l452xx::peripherals;

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

uint32_t get_index_from_Id(GPIO::Id a_id)
{
    return static_cast<uint32_t>(a_id);
}

struct Controller
{
    GPIO_TypeDef* p_registers = nullptr;

    void(*p_enable)()  = nullptr;
    void(*p_disable)() = nullptr;
};

Controller controllers[] =
{
    { GPIOA, gpio_a_enable, gpio_a_disable },
    { GPIOB, gpio_b_enable, gpio_b_disable },
    { GPIOC, gpio_c_enable, gpio_c_disable },
    { GPIOD, gpio_d_enable, gpio_d_disable },
    { GPIOE, gpio_e_enable, gpio_e_disable },
    { nullptr, nullptr, nullptr            },
    { nullptr, nullptr, nullptr            },
    { GPIOH, gpio_h_enable, gpio_h_disable },
};

} // namespace ::

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;

void GPIO::enable()
{
    this->p_gpio = controllers[get_index_from_Id(this->id)].p_registers;
    controllers[get_index_from_Id(this->id)].p_enable();

    set_bit(&(this->flags), 31u);
}

void GPIO::disable()
{
    controllers[get_index_from_Id(this->id)].p_disable();

    clear_bit(&(this->flags), 31);
}

void pin::In::set_pull(Pull a_pull)
{
    assert(Pull::unknown != a_pull);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
             0x3u << (this->id * 2),
             static_cast<uint32_t>(a_pull) << (this->id * 2));
}

pin::Level pin::In::get_level() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Level>(is_bit(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

pin::Pull pin::In::get_pull() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>((get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR, 
                                       static_cast<uint32_t>(0x3u << this->id)) << this->id));
}

void pin::Out::set_level(Level a_level)
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    constexpr uint8_t mul[] = { 16, 0 };
    static_cast<GPIO_TypeDef*>(*(this->p_port))->BSRR = 0x1u << (this->id + mul[static_cast<uint32_t>(a_level)]);
}

void pin::Out::toggle_level()
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    toggle_bit(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->ODR), this->id);
}

void pin::Out::set_mode(Mode a_mode)
{
    assert(Mode::unknown != a_mode);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
             0x1u << this->id,
             static_cast<uint32_t>(a_mode) << this->id);
}

void pin::Out::set_pull(Pull a_pull)
{
    assert(Pull::unknown != a_pull);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
             0x3u << (this->id * 2),
             static_cast<uint32_t>(a_pull) << (this->id * 2));
}

void pin::Out::set_speed(Speed a_speed)
{
    assert(Speed::unknown != a_speed);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR), 0x3u << (this->id * 2),
             static_cast<uint32_t>(a_speed) << (this->id * 2));
}

pin::Level pin::Out::get_level() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Level>(is_bit(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

pin::Mode pin::Out::get_mode() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Mode>(get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                      static_cast<uint32_t>(0x1u << this->id) << this->id));
}

pin::Pull pin::Out::get_pull() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                      static_cast<uint32_t>(0x1u << this->id) << this->id));
}

pin::Speed pin::Out::get_speed() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Speed>((get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR, 
                                        static_cast<uint32_t>(0x1u << this->id) << this->id)));
}

void pin::Analog::set_pull(Pull a_pull)
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
             0x3u << (this->id * 2),
             static_cast<uint32_t>(a_pull) << (this->id * 2));
}

pin::Pull pin::Analog::get_pull() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                      static_cast<uint32_t>(0x1u << this->id) << this->id));
}

void pin::Af::set_mode(Mode a_mode)
{
    assert(Mode::unknown != a_mode);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
             0x1u << this->id,
             static_cast<uint32_t>(a_mode) << this->id);
}

void pin::Af::set_pull(Pull a_pull)
{
    assert(Pull::unknown != a_pull);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
             0x3u << (this->id * 2),
             static_cast<uint32_t>(a_pull) << (this->id * 2));
}

void pin::Af::set_speed(Speed a_speed)
{
    assert(Speed::unknown != a_speed);
    assert(nullptr != this->p_port && 0xFF != this->id);

    set_flag(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
             0x3u << (this->id * 2),
             static_cast<uint32_t>(a_speed) << (this->id * 2));
}

void pin::Af::set_function(uint32_t a_function)
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    uint32_t af_register_index = this->id >> 3u;
    uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((this->id - (af_register_index * 8u)) * 4u));
    af_register |= a_function << ((this->id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;
    this->function = a_function;
}

pin::Mode pin::Af::get_mode() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Mode>(get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                             static_cast<uint32_t>(0x1u << this->id) << this->id));
}

pin::Pull pin::Af::get_pull() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                      static_cast<uint32_t>(0x1u << this->id) << this->id));
}

pin::Speed pin::Af::get_speed() const
{
    assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Speed>(get_flag(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                       static_cast<uint32_t>(0x1u << this->id) << this->id));
}

void pin::in::enable(GPIO* a_p_port, uint32_t a_id, Pull a_pull, In* a_p_out_pin)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());
    assert(false == a_p_port->is_pin_taken(a_id));

    assert(Pull::unknown != a_pull);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*a_p_port));

    set_flag(&(p_port->PUPDR), 0x3u << (a_id * 2), static_cast<uint32_t>(a_pull) << (a_id * 2));
    clear_flag(&(static_cast<GPIO_TypeDef*>(*(a_p_port))->MODER), 0x3u << (a_id * 2));

    a_p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id     = a_id;
        a_p_out_pin->p_port = a_p_port;
    }
}

void pin::in::disable(GPIO* a_p_port, uint32_t a_id)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(a_p_port));

    const uint32_t flag = (0x3u << (a_id * 2));

    set_flag(&(p_port->MODER),   flag);
    clear_flag(&(p_port->PUPDR), flag);

    a_p_port->give_pin(a_id);
}

void pin::out::enable(GPIO* a_p_port, uint32_t a_id, const Config& a_config, Out* a_p_out_pin)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());
    assert(false == a_p_port->is_pin_taken(a_id));

    assert(Pull::unknown  != a_config.pull);
    assert(Speed::unknown != a_config.speed);
    assert(Mode::unknown  != a_config.mode);

    const uint32_t clear_flag_2bit = 0x3u << (a_id * 2);
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(a_p_port));

    set_flag(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<uint32_t>(a_config.speed) << (a_id * 2));
    set_flag(&(p_port->PUPDR),   clear_flag_2bit, static_cast<uint32_t>(a_config.pull) << (a_id * 2));
    set_flag(&(p_port->MODER),   clear_flag_2bit, 0x1u << (a_id * 2));
    set_flag(&(p_port->OTYPER),  0x1u << a_id, static_cast<uint32_t>(a_config.mode) << a_id);

    a_p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id     = a_id;
        a_p_out_pin->p_port = a_p_port;
    }
}

void pin::out::disable(GPIO* a_p_port, uint32_t a_id)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(a_p_port));

    const uint32_t flag = (0x3u << (a_id * 2));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    a_p_port->give_pin(a_id);
}

void pin::analog::enable(GPIO* a_p_port, uint32_t a_id, Pull a_pull, Analog* a_p_out_pin)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());
    assert(false == a_p_port->is_pin_taken(a_id));

    assert(Pull::unknown != a_pull);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(a_p_port));

    set_flag(&(p_port->PUPDR), 0x3u << (a_id * 2), static_cast<uint32_t>(a_pull) << (a_id * 2));
    set_flag(&(p_port->MODER), 0x3u << (a_id * 2), 0x3u << (a_id * 2));

    a_p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id     = a_id;
        a_p_out_pin->p_port = a_p_port;
    }
}

void pin::analog::disable(GPIO* a_p_port, uint32_t a_id)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());

    clear_flag(&(static_cast<GPIO_TypeDef*>(*(a_p_port))->PUPDR), (0x3u << (a_id * 2)));

    a_p_port->give_pin(a_id);
}

void pin::af::enable(GPIO* a_p_port, uint32_t a_id, const Config& a_config, Af* a_p_out_pin)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());
    assert(false == a_p_port->is_pin_taken(a_id));

    assert(Pull::unknown  != a_config.pull);
    assert(Speed::unknown != a_config.speed);
    assert(Mode::unknown  != a_config.mode);

    const uint32_t clear_flag_2bit = 0x3u << (a_id * 2);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(a_p_port));

    set_flag(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<uint32_t>(a_config.speed) << (a_id * 2));
    set_flag(&(p_port->PUPDR),   clear_flag_2bit, static_cast<uint32_t>(a_config.pull)  << (a_id * 2));
    set_flag(&(p_port->MODER),   clear_flag_2bit, 0x2u << (a_id * 2));
    set_flag(&(p_port->OTYPER),  0x1u << a_id,   static_cast<uint32_t>(a_config.mode) << a_id);

    uint32_t af_register_index = a_id >> 3u;
    uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((a_id - (af_register_index * 8u)) * 4u));
    af_register |= a_config.function << ((a_id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;

    a_p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id       = a_id;
        a_p_out_pin->p_port   = a_p_port;
        a_p_out_pin->function = a_config.function;
    }
}

void pin::af::disable(GPIO* a_p_port, uint32_t a_id)
{
    assert(nullptr != a_p_port);
    assert(a_id < 16);
    assert(true == a_p_port->is_enabled());

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(a_p_port));

    const uint32_t flag = (0x3u << (a_id * 2));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    a_p_port->give_pin(a_id);
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx