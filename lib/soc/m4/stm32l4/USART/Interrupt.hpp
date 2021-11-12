#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
#include <soc/m4/stm32l4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>
#include <soc/m4/stm32l4/USART/RS485.hpp>
#include <soc/m4/stm32l4/USART/USART.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<USART> : private cml::Non_copyable
{
public:
    class TX : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(volatile uint16_t* a_p_data, bool a_transfer_complete, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        TX(Interrupt<USART>* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

    private:
        Interrupt<USART>* p_owner;
        Callback callback;

    public:
        friend void usart_interrupt_handler(Interrupt<USART>* a_p_this);

    private:
        friend Interrupt<USART>;
    };

    class RX : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(std::uint32_t a_data, bool a_idle, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        RX(Interrupt<USART>* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

    private:
        Interrupt<USART>* p_owner;
        Callback callback;

    private:
        friend void usart_interrupt_handler(Interrupt<USART>* a_p_this);

    private:
        friend Interrupt<USART>;
    };

    class Status : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            enum class Flag : std::uint32_t
            {
                ok             = 0x0u,
                framing_error  = 0x1u,
                parity_error   = 0x2u,
                overrun        = 0x4u,
                noise_detected = 0x8u,
            };

            using Function = void (*)(Flag a_status, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        Status(Interrupt<USART>* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

    private:
        Interrupt<USART>* p_owner;
        Callback callback;

    private:
        friend void usart_interrupt_handler(Interrupt<USART>* a_p_this);

    private:
        friend Interrupt<USART>;
    };

public:
    ~Interrupt()
    {
        this->disable();
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    USART* get_handle()
    {
        return this->p_USART;
    }

    const USART* get_handle() const
    {
        return this->p_USART;
    }

public:
    TX tx;
    RX rx;
    Status status;

private:
    Interrupt(USART* a_p_USART, IRQn_Type a_irqn)
        : tx(this)
        , rx(this)
        , status(this)
        , p_USART(a_p_USART)
        , irqn(a_irqn)
    {
    }

    void set_irq_context();
    void clear_irq_context();

private:
    USART* p_USART;
    const IRQn_Type irqn;

private:
    template<typename Periph_t, std::size_t id> friend class Factory;
};

template<> class Interrupt<RS485> : cml::Non_copyable
{
public:
    class TX : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(volatile uint16_t* a_p_data, bool a_transfer_complete, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

    public:
        void register_callback(const Callback& a_callback, GPIO::Out::Pin* a_p_flow_control_pin);
        void unregister_callback(GPIO::Out::Pin* a_p_flow_control_pin);

    private:
        TX(Interrupt<RS485>* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

    private:
        Interrupt<RS485>* p_owner;
        Callback callback;

    private:
        friend void rs485_interrupt_handler(Interrupt<RS485>* a_p_this);

    private:
        friend Interrupt<RS485>;
    };

    class RX : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(std::uint32_t a_data,
                                      bool a_idle,
                                      void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

    public:
        void register_callback(const Callback& a_callback, GPIO::Out::Pin* a_p_flow_control_pin);
        void unregister_callback(GPIO::Out::Pin* a_p_flow_control_pin);

    private:
        RX(Interrupt<RS485>* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

    private:
        Interrupt<RS485>* p_owner;
        Callback callback;

    private:
        friend void rs485_interrupt_handler(Interrupt<RS485>* a_p_this);

    private:
        friend class Interrupt<RS485>;
    };

    class Status : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            enum class Flag : std::uint32_t
            {
                ok             = 0x0u,
                framing_error  = 0x1u,
                parity_error   = 0x2u,
                overrun        = 0x4u,
                noise_detected = 0x8u,
            };

            using Function = void (*)(Flag a_status, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

    public:
        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        Status(Interrupt<RS485>* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

    private:
        Callback callback;
        Interrupt<RS485>* p_owner;

    private:
        friend void rs485_interrupt_handler(Interrupt<RS485>* a_p_this);

    private:
        friend class Interrupt<RS485>;
    };

public:
    ~Interrupt()
    {
        this->disable();
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    RS485* get_handle()
    {
        return this->p_RS485;
    }

    const RS485* get_handle() const
    {
        return this->p_RS485;
    }

public:
    TX tx;
    RX rx;
    Status status;

private:
    Interrupt(RS485* a_p_RS485, IRQn_Type a_irqn)
        : tx(this)
        , rx(this)
        , status(this)
        , p_RS485(a_p_RS485)
        , irqn(a_irqn)
    {
    }

    void set_irq_context();
    void clear_irq_context();

private:
    RS485* p_RS485;
    IRQn_Type irqn;

private:
    template<typename Periph_t, std::size_t id> friend class Factory;
};

void usart_interrupt_handler(Interrupt<USART>* a_p_this);
void rs485_interrupt_handler(Interrupt<RS485>* a_p_this);

constexpr Interrupt<USART>::Status::Callback::Flag operator|(Interrupt<USART>::Status::Callback::Flag a_f1,
                                                             Interrupt<USART>::Status::Callback::Flag a_f2)
{
    return static_cast<Interrupt<USART>::Status::Callback::Flag>(static_cast<std::uint32_t>(a_f1) |
                                                                 static_cast<std::uint32_t>(a_f2));
}

constexpr Interrupt<USART>::Status::Callback::Flag operator&(Interrupt<USART>::Status::Callback::Flag a_f1,
                                                             Interrupt<USART>::Status::Callback::Flag a_f2)
{
    return static_cast<Interrupt<USART>::Status::Callback::Flag>(static_cast<std::uint32_t>(a_f1) &
                                                                 static_cast<std::uint32_t>(a_f2));
}

constexpr Interrupt<USART>::Status::Callback::Flag operator|=(Interrupt<USART>::Status::Callback::Flag& a_f1,
                                                              Interrupt<USART>::Status::Callback::Flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr Interrupt<RS485>::Status::Callback::Flag operator|(Interrupt<RS485>::Status::Callback::Flag a_f1,
                                                             Interrupt<RS485>::Status::Callback::Flag a_f2)
{
    return static_cast<Interrupt<RS485>::Status::Callback::Flag>(static_cast<std::uint32_t>(a_f1) |
                                                                 static_cast<std::uint32_t>(a_f2));
}

constexpr Interrupt<RS485>::Status::Callback::Flag operator&(Interrupt<RS485>::Status::Callback::Flag a_f1,
                                                             Interrupt<RS485>::Status::Callback::Flag a_f2)
{
    return static_cast<Interrupt<RS485>::Status::Callback::Flag>(static_cast<std::uint32_t>(a_f1) &
                                                                 static_cast<std::uint32_t>(a_f2));
}

constexpr Interrupt<RS485>::Status::Callback::Flag operator|=(Interrupt<RS485>::Status::Callback::Flag& a_f1,
                                                              Interrupt<RS485>::Status::Callback::Flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc