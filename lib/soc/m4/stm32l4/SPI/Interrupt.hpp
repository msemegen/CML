#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <limits>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/Interrupt_guard.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>
#include <soc/m4/stm32l4/SPI/SPI.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

class SPI_transmission_interrupt : private cml::Non_copyable
{
public:
    class TX
    {
    public:
        struct Callback
        {
            using Function = void (*)(volatile std::uint16_t* a_p_data, bool a_stop, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        TX(TX&&)    = default;
        TX& operator=(TX&&) = default;

        TX()
            : p_registers(nullptr)
        {
        }

        ~TX()
        {
            this->unregister_callback();
        }

        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        TX(SPI_TypeDef* a_p_registers)
            : p_registers(a_p_registers)
        {
        }

        SPI_TypeDef* p_registers;
        Callback callback;

        friend SPI_transmission_interrupt;
        friend void SPI_interrupt_handler(SPI_transmission_interrupt::TX* a_p_this);
    };
    class RX
    {
    public:
        struct Callback
        {
            using Function = void (*)(std::uint16_t a_data, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        RX(RX&&)    = default;
        RX& operator=(RX&&) = default;

        RX()
            : p_registers(nullptr)
        {
        }

        ~RX()
        {
            this->unregister_callback();
        }

        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        RX(SPI_TypeDef* a_p_registers)
            : p_registers(a_p_registers)
        {
        }

        SPI_TypeDef* p_registers;
        Callback callback;

        friend SPI_transmission_interrupt;
        friend void SPI_interrupt_handler(SPI_transmission_interrupt::RX* a_p_this);
    };

    SPI_transmission_interrupt() {}

    TX tx;
    RX rx;

private:
    SPI_transmission_interrupt(SPI_TypeDef* a_p_registers)
        : tx(a_p_registers)
        , rx(a_p_registers)
    {
    }

    friend class Interrupt<SPI>;
};

class SPI_status_interrupt : private cml::Non_copyable
{
public:
    struct Callback
    {
        enum class Flag : std::uint32_t
        {
            ok          = 0x0u,
            overrun     = 0x1u,
            crc_error   = 0x2u,
            frame_error = 0x4u,
            mode_fault  = 0x8u,
        };

        using Function    = void (*)(Flag a_flag, void* a_p_user_data);
        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    void register_callback(const Callback& a_callback);
    void unregister_callback();

    SPI_status_interrupt()
        : p_registers(nullptr)
    {
    }

    ~SPI_status_interrupt() {}

private:
    SPI_status_interrupt(SPI_TypeDef* a_p_registers)
        : p_registers(a_p_registers)
    {
    }

    SPI_TypeDef* p_registers;
    Callback callback;

    friend class Interrupt<SPI>;
    friend void SPI_interrupt_handler(SPI_status_interrupt* a_p_this);
};

template<> class Interrupt<SPI> : private cml::Non_copyable
{
public:
    using Transmission = SPI_transmission_interrupt;
    using Status       = SPI_status_interrupt;

    Transmission transmission;
    Status status;

    Interrupt()
        : irqn(static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()))
    {
    }

protected:
    Interrupt(SPI_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : transmission(a_p_registers)
        , status(a_p_registers)
        , irqn(a_irqn)
    {
    }

    const IRQn_Type irqn;
};

template<> class Interrupt<SPI_master> : public Interrupt<SPI>
{
public:
    Interrupt()
        : p_SPI(nullptr)
    {
    }

    ~Interrupt()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    bool is_enabled() const
    {
        return 0u != NVIC_GetEnableIRQ(this->irqn);
    }

    bool is_created() const
    {
        return nullptr != this->p_SPI;
    }

    SPI_master* get_handle()
    {
        return this->p_SPI;
    }

    const SPI_master* get_handle() const
    {
        return this->p_SPI;
    }

private:
    Interrupt(SPI_master* a_p_SPI, IRQn_Type a_irqn)
        : Interrupt<SPI>(*a_p_SPI, a_irqn)
        , p_SPI(a_p_SPI)
    {
    }

    void set_irq_context();
    void clear_irq_context();

    SPI_master* p_SPI;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

template<> class Interrupt<SPI_slave> : private Interrupt<SPI>
{
public:
    Interrupt()
        : p_SPI(nullptr)
    {
    }

    ~Interrupt()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    bool is_enabled() const
    {
        return 0u != NVIC_GetEnableIRQ(this->irqn);
    }

    bool is_created() const
    {
        return nullptr != this->p_SPI;
    }

    SPI_slave* get_handle()
    {
        return this->p_SPI;
    }

    const SPI_slave* get_handle() const
    {
        return this->p_SPI;
    }

private:
    Interrupt(SPI_slave* a_p_SPI, IRQn_Type a_irqn)
        : Interrupt<SPI>(*a_p_SPI, a_irqn)
        , p_SPI(a_p_SPI)
    {
    }

    void set_irq_context();
    void clear_irq_context();

    SPI_slave* p_SPI;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

constexpr SPI_status_interrupt::Callback::Flag operator|(SPI_status_interrupt::Callback::Flag a_f1,
                                                         SPI_status_interrupt::Callback::Flag a_f2)
{
    return static_cast<SPI_status_interrupt::Callback::Flag>(static_cast<std::uint32_t>(a_f1) |
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr SPI_status_interrupt::Callback::Flag operator&(SPI_status_interrupt::Callback::Flag a_f1,
                                                         SPI_status_interrupt::Callback::Flag a_f2)
{
    return static_cast<SPI_status_interrupt::Callback::Flag>(static_cast<std::uint32_t>(a_f1) &
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr SPI_status_interrupt::Callback::Flag operator|=(SPI_status_interrupt::Callback::Flag& a_f1,
                                                          SPI_status_interrupt::Callback::Flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc