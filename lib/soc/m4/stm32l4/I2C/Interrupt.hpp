#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/I2C/I2C.hpp>
#include <soc/m4/stm32l4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>

// cml
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<I2C_slave>;
template<> class Interrupt<I2C_master>;

class I2C_status_interrupt : private cml::Non_copyable
{
public:
    struct Callback
    {
        enum class Flag : std::uint32_t
        {
            ok               = 0x0,
            crc_error        = 0x1,
            buffer_error     = 0x2,
            arbitration_lost = 0x4,
            misplaced        = 0x8,
            nack             = 0x10
        };

        using Function = void (*)(Flag a_flag, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    ~I2C_status_interrupt()
    {
        this->disable();
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    void register_callback(const Callback& a_callback);
    void unregister_callback();

private:
    I2C_status_interrupt(I2C_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : p_registers(a_p_registers)
        , irqn(a_irqn)
    {
    }

    I2C_TypeDef* p_registers;
    Callback callback;
    IRQn_Type irqn;

    friend class Interrupt<I2C>;
    friend void I2C_interrupt_handler(I2C_status_interrupt* a_p_this);
};

class I2C_TX_interrupt : private cml::Non_copyable
{
public:
    struct Callback
    {
        using Function = void (*)(volatile std::uint32_t* a_p_data, bool a_stop, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

protected:
    I2C_TX_interrupt(I2C_TypeDef* a_p_registers)
        : p_registers(a_p_registers)
    {
    }

    I2C_TypeDef* p_registers;
    Callback callback;

    friend void I2C_interrupt_handler(I2C_TX_interrupt* a_p_this);
};

class I2C_RX_interrupt : private cml::Non_copyable
{
public:
    struct Callback
    {
        using Function = void (*)(std::uint8_t a_data, bool a_stop, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

protected:
    I2C_RX_interrupt(I2C_TypeDef* a_p_registers)
        : p_registers(a_p_registers)
    {
    }

    I2C_TypeDef* p_registers;
    Callback callback;

    friend void I2C_interrupt_handler(I2C_RX_interrupt* a_p_this);
};

template<> class Interrupt<I2C> : private cml::Non_copyable
{
public:
    using Status = I2C_status_interrupt;

    Status status;

protected:
    Interrupt(I2C_TypeDef* a_p_registers, IRQn_Type a_er_irqn)
        : status(a_p_registers, a_er_irqn)
    {
    }

    I2C_TX_interrupt* p_tx;
    I2C_RX_interrupt* p_rx;

    friend void I2C_interrupt_handler(Interrupt<I2C>* a_p_this);
};

template<> class Interrupt<I2C_master> : public Interrupt<I2C>
{
public:
    class Transmission : private cml::Non_copyable
    {
    public:
        class TX : public I2C_TX_interrupt
        {
        public:
            void register_callback(std::uint8_t a_slave_address,
                                   std::size_t a_data_length_in_bytes,
                                   const Callback& a_callback);
            void unregister_callback();

            ~TX()
            {
                this->unregister_callback();
            }

        private:
            TX(I2C_TypeDef* a_p_registers)
                : I2C_TX_interrupt(a_p_registers)
            {
            }

            friend class Transmission;
        };
        class RX : public I2C_RX_interrupt
        {
        public:
            void register_callback(std::uint8_t a_slave_address,
                                   std::size_t a_data_length_in_bytes,
                                   const Callback& a_callback);
            void unregister_callback();

            ~RX()
            {
                this->unregister_callback();
            }

        private:
            RX(I2C_TypeDef* a_p_registers)
                : I2C_RX_interrupt(a_p_registers)
            {
            }

            friend class Transmission;
        };

        ~Transmission()
        {
            this->disable();
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        TX tx;
        RX rx;

    private:
        Transmission(I2C_TypeDef* a_p_registers, IRQn_Type a_irqn)
            : tx(a_p_registers)
            , rx(a_p_registers)
        {
        }

        IRQn_Type irqn;

        friend Interrupt<I2C_master>;
    };

    ~Interrupt();

    I2C_master* get_handle()
    {
        return this->p_I2C;
    }

    const I2C_master* get_handle() const
    {
        return this->p_I2C;
    }

    Transmission transmission;

private:
    Interrupt(I2C_master* a_p_I2C, IRQn_Type a_ev_irqn, IRQn_Type a_er_irqn);

    I2C_master* p_I2C;

    template<typename Periph_t, std::size_t id> friend class Factory;
};

template<> class Interrupt<I2C_slave> : public Interrupt<I2C>
{
public:
    class Transmission : private cml::Non_copyable
    {
    public:
        class TX : public I2C_TX_interrupt
        {
        public:
            void register_callback(const Callback& a_callback);
            void unregister_callback();

            ~TX()
            {
                this->unregister_callback();
            }

        private:
            TX(I2C_TypeDef* a_p_registers)
                : I2C_TX_interrupt(a_p_registers)
            {
            }

            friend class Transmission;
        };
        class RX : public I2C_RX_interrupt
        {
        public:
            void register_callback(const Callback& a_callback);
            void unregister_callback();

            ~RX()
            {
                this->unregister_callback();
            }

        private:
            RX(I2C_TypeDef* a_p_registers)
                : I2C_RX_interrupt(a_p_registers)
            {
            }

            friend class Transmission;
        };

        ~Transmission()
        {
            this->disable();
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        TX tx;
        RX rx;

    private:
        Transmission(I2C_TypeDef* a_p_registers, IRQn_Type a_irqn)
            : tx(a_p_registers)
            , rx(a_p_registers)
        {
        }

        IRQn_Type irqn;

        friend Interrupt<I2C_slave>;
    };

    ~Interrupt();

    I2C_slave* get_handle()
    {
        return this->p_I2C;
    }

    const I2C_slave* get_handle() const
    {
        return this->p_I2C;
    }

    Transmission transmission;

private:
    Interrupt(I2C_slave* a_p_I2C, IRQn_Type a_ev_irqn, IRQn_Type a_er_irqn);

    I2C_slave* p_I2C;

    template<typename Periph_t, std::size_t id> friend class Factory;
};

constexpr I2C_status_interrupt::Callback::Flag operator|(I2C_status_interrupt::Callback::Flag a_f1,
                                                         I2C_status_interrupt::Callback::Flag a_f2)
{
    return static_cast<I2C_status_interrupt::Callback::Flag>(static_cast<std::uint32_t>(a_f1) |
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr I2C_status_interrupt::Callback::Flag operator&(I2C_status_interrupt::Callback::Flag a_f1,
                                                         I2C_status_interrupt::Callback::Flag a_f2)
{
    return static_cast<I2C_status_interrupt::Callback::Flag>(static_cast<std::uint32_t>(a_f1) &
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr I2C_status_interrupt::Callback::Flag operator|=(I2C_status_interrupt::Callback::Flag& a_f1,
                                                          I2C_status_interrupt::Callback::Flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc