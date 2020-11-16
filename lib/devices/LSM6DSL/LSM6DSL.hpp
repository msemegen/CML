#pragma once

/*
    Name: LSM6DSL.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace cml {
namespace devices {

class LSM6DSL
{
public:
    struct Fifo_config
    {
        bool enabled                     = false;
        uint8_t pattern_repetition_count = 0;
    };

    struct Accelerometer_config
    {
        enum class Fullscale_range : uint8_t
        {
            _2g     = 0x0u << 2u,
            _16g    = 0x1u << 2u,
            _4g     = 0x2u << 2u,
            _8g     = 0x3u << 2u,
            unknown = 0xFFu
        };

        enum class Output_data_rate : uint8_t
        {
            _12_5_Hz = 0x1u,
            _26_Hz   = 0x2u,
            _52_Hz   = 0x3u,
            _104_Hz  = 0x4u,
            _208_Hz  = 0x5u,
            _416_Hz  = 0x6u,
            _833_Hz  = 0x7u,
            _1660_Hz = 0x8u,
            _3330_Hz = 0x9u,
            _6660_Hz = 0xAu,
            _1_6_Hz  = 0xBu,
            unknown  = 0xFFu
        };

        bool enabled                      = false;
        Fullscale_range fullscale_range   = Fullscale_range::unknown;
        Output_data_rate output_data_rate = Output_data_rate::unknown;
    };

    struct Gyroscope_config
    {
        enum class Fullscale_range : uint8_t
        {
            _125_dps  = 0x1u << 1u,
            _250_dps  = 0x0u << 1u,
            _500_dps  = 0x2u << 1u,
            _1000_dps = 0x4u << 1u,
            _2000_dps = 0x6u << 1u,
            unknown   = 0xFFu
        };

        enum class Output_data_rate : uint8_t
        {
            _12_5_Hz = 0x1u,
            _26_Hz   = 0x2u,
            _52_Hz   = 0x3u,
            _104_Hz  = 0x4u,
            _208_Hz  = 0x5u,
            _416_Hz  = 0x6u,
            _833_Hz  = 0x7u,
            _1660_Hz = 0x8u,
            _3330_Hz = 0x9u,
            _6660_Hz = 0xAu,
            unknown  = 0xFFu
        };

        bool enabled                      = false;
        Fullscale_range fullscale_range   = Fullscale_range::unknown;
        Output_data_rate output_data_rate = Output_data_rate::unknown;
    };

    struct Transmit
    {
        using Function = bool (*)(uint8_t a_register, uint8_t a_value, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive
    {
        using Function = bool (*)(uint8_t a_register, uint8_t* a_p_data, uint32_t a_size_in_bytes, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    template<typename Type_t> struct Axis
    {
        union
        {
            struct
            {
                Type_t x = static_cast<Type_t>(0);
                Type_t y = static_cast<Type_t>(0);
                Type_t z = static_cast<Type_t>(0);
            };

            Type_t linear[3];
        };
    };

public:
    LSM6DSL(Transmit a_transmit, Receive a_receive)
        : transmit(a_transmit)
        , receive(a_receive)
    {
    }
    ~LSM6DSL() = default;

    bool enable(const Accelerometer_config& a_acc_config,
                const Gyroscope_config& a_gyr_config,
                const Fifo_config& a_fifo_config);

    bool disable(bool a_accelerometer, bool a_groscope);

    bool is_alive() const;
    bool reset();

    bool get_axis_raw(Axis<int16_t>* a_p_gyr_data,
                      uint32_t a_gyr_data_buffer_capacity,
                      Axis<int16_t>* a_p_acc_data,
                      uint32_t a_acc_data_buffer_capacity);

    bool get_axis_scaled(Axis<float>* a_p_gyr_data,
                         uint32_t a_gyr_data_buffer_capacity,
                         Axis<float>* a_p_acc_data,
                         uint32_t a_acc_data_buffer_capacity);

    const Accelerometer_config& get_accelerometer_config() const
    {
        return this->accelerometer_config;
    }

    const Gyroscope_config& get_gyroscope_config() const
    {
        return this->gyroscope_config;
    }

    const Fifo_config& get_fifo_config() const
    {
        return this->fifo_config;
    }

private:
    struct Registers
    {
        enum
        {
            fifo_ctrl1      = 0x6u,
            fifo_ctrl2      = 0x7u,
            fifo_ctrl3      = 0x8u,
            fifo_ctrl5      = 0xAu,
            int1_ctrl       = 0xDu,
            int2_ctrl       = 0xEu,
            ctrl1_xl        = 0x10u,
            ctrl2_g         = 0x11u,
            ctrl3_c         = 0x12u,
            ctrl6_c         = 0x15u,
            ctrl7_g         = 0x16u,
            outx_l_g        = 0x22u,
            outx_l_xl       = 0x28u,
            status_reg      = 0x1Eu,
            fifo_status2    = 0x3Bu,
            fifo_data_out_l = 0x3Eu,
        };
    };

    struct Fifo_context
    {
        uint8_t gyr_decimation    = 0;
        uint8_t acc_decimation    = 0;
        uint8_t gyr_samples_count = 0;
        uint8_t acc_samples_count = 0;
        uint8_t odr               = 0;
    };

private:
    bool set_fifo_context(const Fifo_context& a_context);
    bool get_axis(uint8_t a_outx_register, int16_t* a_p_outx, int16_t* a_p_outy, int16_t* a_p_outz);

    bool is_fifo_full(bool* a_p_status) const;

    bool is_accelerometer_data_avaliable(bool* a_p_status) const;
    bool is_gyroscope_data_avaliable(bool* a_p_status) const;

private:
    Transmit transmit;
    Receive receive;

    Accelerometer_config accelerometer_config;
    Gyroscope_config gyroscope_config;
    Fifo_config fifo_config;

    Fifo_context fifo_context;
};

} // namespace devices
} // namespace cml