#pragma once

/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class I2C : private cml::Non_copyable
{
public:
    enum class Event_flag : std::uint32_t
    {
        ok               = 0x0,
        crc_error        = 0x1,
        buffer_error     = 0x2,
        arbitration_lost = 0x4,
        misplaced        = 0x8,
        nack             = 0x10,
    };

    class Interrupt
    {
    public:
        struct Transmit_callback
        {
            using Function = void (*)(volatile std::uint32_t* a_p_data, bool a_stop, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        struct Receive_callback
        {
            using Function = void (*)(std::uint8_t a_data, bool a_stop, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        struct Event_callback
        {
            using Function = void (*)(Event_flag a_flag, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_irq_transceiving_config, const IRQ_config& a_irq_event_config);
        void disable();

        void transmit_start(const Transmit_callback& a_callback);
        void transmit_stop();

        void receive_start(const Receive_callback& a_callback);
        void receive_stop();

        void event_listening_start(const Event_callback& a_callback, Event_flag a_enabled_events);
        void event_listening_stop();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_I2C->ev_irqn) && 0 != NVIC_GetEnableIRQ(this->p_I2C->er_irqn);
        }

    private:
        void set_irq_context();
        void clear_irq_context();

        I2C* p_I2C = nullptr;
        friend class I2C;
    };

    I2C()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , ev_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , er_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    ~I2C()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void disable();

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    bool is_enabled() const
    {
        return cml::bit_flag::is(this->p_registers->CR1, I2C_CR1_PE);
    }

    bool is_created() const
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    operator I2C_TypeDef*()
    {
        return this->p_registers;
    }

    operator const I2C_TypeDef*() const
    {
        return this->p_registers;
    }

    Interrupt interrupt;

protected:
    I2C(std::size_t a_idx, I2C_TypeDef* a_p_registers, IRQn_Type a_ev_irqn, IRQn_Type a_er_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , ev_irqn(a_ev_irqn)
        , er_irqn(a_er_irqn)
    {
    }

    const std::uint32_t idx;
    I2C_TypeDef* p_registers;

    IRQn_Type ev_irqn;
    IRQn_Type er_irqn;
};

class I2C_master : public I2C
{
public:
    struct Enable_config
    {
        enum class Analog_filter : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Fast_plus : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Crc : std::uint32_t
        {
            disabled,
            enabled
        };

        Analog_filter analog_filter = cml::various::get_enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::get_enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::get_enum_incorrect_value<Crc>();
        std::uint32_t timings       = 0;
    };

    class Polling
    {
    public:
        struct Result
        {
            Event_flag event                 = cml::various::get_enum_incorrect_value<Event_flag>();
            std::size_t data_length_in_words = 0;
        };

        Result transmit(std::uint8_t a_slave_address, const void* a_p_data, std::size_t a_data_size_in_bytes);
        Result transmit(std::uint8_t a_slave_address,
                        const void* a_p_data,
                        std::size_t a_data_size_in_bytes,
                        cml::Milliseconds a_timeout);

        Result receive(std::uint8_t a_slave_address, void* a_p_data, std::size_t a_data_size_in_bytes);
        Result receive(std::uint8_t a_slave_address,
                       void* a_p_data,
                       std::size_t a_data_size_in_bytes,
                       cml::Milliseconds a_timeout);

    private:
        I2C_master* p_I2C = nullptr;
        friend class I2C_master;
    };

    I2C_master(I2C_master&&) = default;
    I2C_master& operator=(I2C_master&&) = default;

    I2C_master()
        : I2C()
    {
    }

    ~I2C_master()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(const Enable_config& a_config);

    Enable_config get_Enable_config() const;

    Polling polling;

private:
    I2C_master(std::size_t a_idx, I2C_TypeDef* a_p_registers, IRQn_Type a_ev_irqn, IRQn_Type a_er_irqn)
        : I2C(a_idx, a_p_registers, a_ev_irqn, a_er_irqn)
    {
    }

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

class I2C_slave : public I2C
{
public:
    struct Enable_config
    {
        enum class Analog_filter : std::uint32_t
        {
            disabled,
            enabled,
        };

        enum class Fast_plus : std::uint32_t
        {
            disabled,
            enabled,
        };

        enum class Crc : std::uint32_t
        {
            disabled,
            enabled,
        };

        Analog_filter analog_filter = cml::various::get_enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::get_enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::get_enum_incorrect_value<Crc>();
        std::uint32_t timings       = 0;
        std::uint16_t address       = 0;
    };

    class Polling
    {
    public:
        struct Result
        {
            Event_flag event                 = cml::various::get_enum_incorrect_value<Event_flag>();
            std::size_t data_length_in_words = 0;
        };

        Result transmit(const void* a_p_data, std::size_t a_data_size_in_bytes);
        Result transmit(const void* a_p_data, std::size_t a_data_size_in_bytes, cml::Milliseconds a_timeout);

        Result receive(void* a_p_data, std::size_t a_data_size_in_bytes);
        Result receive(void* a_p_data, std::size_t a_data_size_in_bytes, cml::Milliseconds a_timeout);

    private:
        I2C_slave* p_I2C = nullptr;
        friend class I2C_slave;
    };

    I2C_slave(I2C_slave&&) = default;
    I2C_slave& operator=(I2C_slave&&) = default;

    I2C_slave()
        : I2C()
    {
    }

    ~I2C_slave()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(const Enable_config& a_config);

    Enable_config get_Enable_config() const;

    Polling polling;

private:
    I2C_slave(std::size_t a_idx, I2C_TypeDef* a_p_registers, IRQn_Type a_ev_irqn, IRQn_Type a_er_irqn)
        : I2C(a_idx, a_p_registers, a_ev_irqn, a_er_irqn)
    {
    }

private:
    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

template<std::size_t id> class rcc<I2C, id> : private cml::Non_constructible
{
public:
    enum class Clock_source : uint32_t
    {
        PCLK1  = 0,
        SYSCLK = 1,
        HSI    = 2
    };

    template<Clock_source> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                          = delete;
};

constexpr I2C_master::Event_flag operator|(I2C_master::Event_flag a_f1, I2C_master::Event_flag a_f2)
{
    return static_cast<I2C_master::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}
constexpr I2C_master::Event_flag operator&(I2C_master::Event_flag a_f1, I2C_master::Event_flag a_f2)
{
    return static_cast<I2C_master::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}
constexpr I2C_master::Event_flag operator|=(I2C_master::Event_flag& a_f1, I2C_master::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc