#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <type_traits>

// cml
#include <cml/Non_copyable.hpp>

// soc
#include <soc/stm32l452xx/peripherals/GPIO.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

class SPI_base : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        _1,
        _2,
        _3
    };

    enum class Bus_flag : uint32_t
    {
        ok          = 0x0u,
        overrun     = 0x1u,
        crc_error   = 0x2u,
        frame_error = 0x4u,
        mode_fault  = 0x8u,
        unknown     = 0x10u
    };

    struct Frame_format
    {
        enum class Word_length : uint32_t
        {
            _4  = 0x300u,
            _5  = 0x400u,
            _6  = 0x500u,
            _7  = 0x600u,
            _8  = 0x700u,
            _9  = 0x800u,
            _10 = 0x900u,
            _11 = 0xA00u,
            _12 = 0xB00u,
            _13 = 0xC00u,
            _14 = 0xD00u,
            _15 = 0xE00u,
            _16 = 0xF00u,
            unknown
        };

        enum class Bit_significance : uint32_t
        {
            most  = 0x0u,
            least = SPI_CR1_LSBFIRST,
            unknown
        };

        enum class Polarity : uint32_t
        {
            high    = SPI_CR1_CPOL,
            low     = 0x0u,
            unknown = 0xFF
        };

        enum class Phase : uint32_t
        {
            first_edge  = 0x0u,
            second_edge = SPI_CR1_CPHA,
            unknown     = 0xFF
        };

        Polarity polarity                 = Polarity::unknown;
        Phase phase                       = Phase::unknown;
        Word_length word_length           = Word_length::unknown;
        Bit_significance bit_significance = Bit_significance::unknown;
    };

    struct Clock_source
    {
        enum class Type
        {
            pclkx,
            unknown
        };

        enum class Prescaler : uint32_t
        {
            _2   = 0x00u,
            _4   = 0x08u,
            _8   = 0x10u,
            _16  = 0x18u,
            _32  = 0x20u,
            _64  = 0x28u,
            _128 = 0x30u,
            _256 = 0x38u,
            unknown
        };

        Type type           = Type::unknown;
        Prescaler prescaler = Prescaler::unknown;
    };

    struct Result
    {
        Bus_flag bus_flag             = Bus_flag::unknown;
        uint32_t data_length_in_words = 0;
    };

    struct Transmit_callback
    {
        using Function = void (*)(volatile uint16_t* a_p_data, bool a_stop, SPI_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive_callback
    {
        using Function = void (*)(uint16_t a_data, SPI_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Transmit_receive_callback
    {
        using Transmit_function = void (*)(volatile uint16_t* a_p_data,
                                           bool a_stop,
                                           SPI_base* a_p_this,
                                           void* a_p_user_data);
        using Receive_function  = void (*)(uint16_t a_data, SPI_base* a_p_this, void* a_p_user_data);

        Transmit_function transmit = nullptr;
        Receive_function receive   = nullptr;
        void* p_user_data          = nullptr;
    };

    struct Bus_status_callback
    {
        using Function    = void (*)(Bus_flag a_flag, SPI_base* a_p_this, void* a_p_user_data);
        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Id get_id() const
    {
        return this->id;
    }

    void register_transmit_callback(const Transmit_callback& a_callback);
    void register_receive_callback(const Receive_callback& a_callback);
    void register_transmit_receive_callaback(const Transmit_receive_callback& a_callback);
    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_transmit_receive_callback();
    void unregister_bus_status_callback();

    Frame_format get_frame_format() const;
    Clock_source get_clock_source() const;

protected:
    SPI_base(Id a_id)
        : id(a_id)
    {
    }

protected:
    Id id;

    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Transmit_receive_callback transmit_receive_callback;
    Bus_status_callback bus_status_callback;

protected:
    friend void spi_interrupt_handler(SPI_base* a_p_this);
};

constexpr SPI_base::Bus_flag operator|(SPI_base::Bus_flag a_f1, SPI_base::Bus_flag a_f2)
{
    return static_cast<SPI_base::Bus_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr SPI_base::Bus_flag operator&(SPI_base::Bus_flag a_f1, SPI_base::Bus_flag a_f2)
{
    return static_cast<SPI_base::Bus_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr SPI_base::Bus_flag operator|=(SPI_base::Bus_flag& a_f1, SPI_base::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

class SPI_master : public SPI_base
{
public:
    using Id           = SPI_base::Id;
    using Frame_format = SPI_base::Frame_format;
    using Clock_source = SPI_base::Clock_source;
    using Result       = SPI_base::Result;

    struct Config
    {
        enum class Wiring : uint32_t
        {
            full_duplex = 0x0000u,
            half_duplex = SPI_CR1_BIDIMODE,
            simplex     = SPI_CR1_RXONLY,
            unknown
        };

        enum class NSS_management : uint32_t
        {
            hardware,
            software = SPI_CR1_SSM | SPI_CR1_SSI,
            unknown
        };

        enum class Crc : uint32_t
        {
            enable  = SPI_CR1_CRCEN,
            disable = 0x0u,
            unknown
        };

        Wiring wiring                 = Wiring::unknown;
        NSS_management nss_management = NSS_management::unknown;
        Crc crc                       = Crc::unknown;
    };

public:
    SPI_master(Id a_id)
        : SPI_base(a_id)
    {
    }

    ~SPI_master()
    {
        this->disable();
    }

    void enable(const Config& a_config,
                const Frame_format& a_frame_format,
                const Clock_source& a_clock_source,
                uint32_t a_irq_priority);
    void disable();

    template<typename Data_t> Result transmit_polling(const Data_t& a_data, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_p_nss);
    }

    template<typename Data_t>
    Result transmit_polling(const Data_t& a_data, uint32_t a_timeout, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout, a_p_nss);
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(*a_p_data), a_p_nss);
    }

    template<typename Data_t>
    Result receive_polling(Data_t* a_p_data, uint32_t a_timeout, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, a_timeout, sizeof(*a_p_data), a_p_nss);
    }

    template<typename Data_t> Result
    transmit_receive_polling(const Data_t& a_transmit_data, Data_t* a_p_receive_data, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_receive_bytes_polling(&a_transmit_data, a_p_receive_data, sizeof(Data_t), a_p_nss);
    }

    template<typename Data_t> Result transmit_receive_polling(const Data_t& a_transmit_data,
                                                              Data_t* a_p_receive_data,
                                                              uint32_t a_timeout,
                                                              GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_receive_bytes_polling(
            &a_transmit_data, a_p_receive_data, sizeof(Data_t), a_timeout, a_p_nss);
    }

    Result
    transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss = nullptr);
    Result transmit_bytes_polling(const void* a_p_data,
                                  uint32_t a_data_size_in_words,
                                  uint32_t a_timeout,
                                  GPIO::Out::Pin* a_p_nss = nullptr);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss = nullptr);
    Result receive_bytes_polling(void* a_p_data,
                                 uint32_t a_data_size_in_words,
                                 uint32_t a_timeout,
                                 GPIO::Out::Pin* a_p_nss = nullptr);

    Result transmit_receive_bytes_polling(const void* a_p_tx_data,
                                          void* a_p_rx_data,
                                          uint32_t a_tx_rx_data_size_in_words,
                                          GPIO::Out::Pin* a_p_nss = nullptr);

    Result transmit_receive_bytes_polling(const void* a_p_tx_data,
                                          void* a_p_rx_data,
                                          uint32_t a_tx_rx_data_size_in_words,
                                          uint32_t a_timeout,
                                          GPIO::Out::Pin* a_p_nss = nullptr);

    Config get_config() const;
};

class SPI_slave : public SPI_base
{
public:
    using Id           = SPI_base::Id;
    using Frame_format = SPI_base::Frame_format;
    using Clock_source = SPI_base::Clock_source;

    struct Config
    {
        enum class Wiring : uint32_t
        {
            full_duplex = 0x0000u,
            half_duplex = SPI_CR1_BIDIMODE,
            simplex     = SPI_CR1_RXONLY,
            unknown
        };

        enum class Crc : uint32_t
        {
            enable  = SPI_CR1_CRCEN,
            disable = 0x0u,
            unknown
        };

        Wiring wiring = Wiring::unknown;
        Crc crc       = Crc::unknown;
    };

public:
    SPI_slave(Id a_id)
        : SPI_base(a_id)
    {
    }

    ~SPI_slave()
    {
        this->disable();
    }

    void enable(const Config& a_config,
                const Frame_format& a_frame_format,
                const Clock_source& a_clock_source,
                uint32_t a_irq_priority);
    void disable();

    template<typename Data_t> Result transmit_polling(const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data));
    }

    template<typename Data_t> Result transmit_polling(const Data_t& a_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(*a_p_data));
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, a_timeout, sizeof(*a_p_data));
    }

    template<typename Data_t> Result transmit_receive_polling(const Data_t& a_transmit_data, Data_t* a_p_receive_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_receive_bytes_polling(&a_transmit_data, a_p_receive_data, sizeof(Data_t));
    }

    template<typename Data_t>
    Result transmit_receive_polling(const Data_t& a_transmit_data, Data_t* a_p_receive_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_receive_bytes_polling(&a_transmit_data, a_p_receive_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words);
    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words);
    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout);

    Result
    transmit_receive_bytes_polling(const void* a_p_tx_data, void* a_p_rx_data, uint32_t a_tx_rx_data_size_in_words);

    Result transmit_receive_bytes_polling(const void* a_p_tx_data,
                                          void* a_p_rx_data,
                                          uint32_t a_tx_rx_data_size_in_words,
                                          uint32_t a_timeout);

    Config get_config() const;
};

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc