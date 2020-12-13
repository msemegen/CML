/*
    Name: LSM6DSL.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <devices/LSM6DSL/LSM6DSL.hpp>

// std
#include <algorithm>

// soc
#include <soc/counter.hpp>

// cml
#include <cml/bit.hpp>

namespace {

constexpr float asl[]                                  = { 0.000061f, 0.000488f, 0.000122f, 0.000244f };
constexpr float gsl[]                                  = { 0.00875, 0.0175f, 0.035f, 0.07f, 0.004375f };
constexpr static uint32_t output_data_rate_to_Hz_lut[] = { 0u,   13u,  26u,   52u,   104u,  208u,
                                                           416u, 833u, 1660u, 3330u, 6660u, 2u };

struct Registers
{
    enum
    {
        fifo_ctrl1      = 0x6u,
        fifo_ctrl2      = 0x7u,
        fifo_ctrl3      = 0x8u,
        fifo_ctrl4      = 0x9u,
        fifo_ctrl5      = 0xAu,
        int1_ctrl       = 0xDu,
        int2_ctrl       = 0xEu,
        ctrl1_xl        = 0x10u,
        ctrl2_g         = 0x11u,
        ctrl3_c         = 0x12u,
        ctrl6_c         = 0x15u,
        ctrl7_g         = 0x16u,
        out_temp_l      = 0x20u,
        outx_l_g        = 0x22u,
        outx_l_xl       = 0x28u,
        status_reg      = 0x1Eu,
        fifo_status1    = 0x3Au,
        fifo_status2    = 0x3Bu,
        fifo_data_out_l = 0x3Eu,
    };
};

} // namespace

namespace cml {
namespace devices {

using namespace soc;

bool LSM6DSL::enable()
{
    bool ret = this->transmit.function(Registers::ctrl3_c, 0x44u, this->transmit.p_user_data);

    if (true == ret)
    {
        ret = this->transmit.function(Registers::ctrl1_xl, 0x0u, this->transmit.p_user_data) &&
              this->transmit.function(Registers::ctrl2_g, 0x0u, this->transmit.p_user_data);
    }

    return ret;
}

bool LSM6DSL::disable()
{
    bool ret = this->transmit.function(Registers::ctrl1_xl, 0x0u, this->transmit.p_user_data) &&
               this->transmit.function(Registers::ctrl2_g, 0x0u, this->transmit.p_user_data);

    if (true == ret)
    {
        ret = this->transmit.function(Registers::ctrl3_c, 0x0u, this->transmit.p_user_data);
    }

    return ret;
}

bool LSM6DSL::reboot(bool* a_p_flag)
{
    *a_p_flag = false;
    bool ret  = this->transmit(Registers::ctrl3_c, 0x80u);

    if (true == ret)
    {
        uint8_t v = 0;
        while (true == ret && false == *a_p_flag)
        {
            ret         = this->receive(Registers::ctrl3_c, &v, sizeof(v));
            (*a_p_flag) = false == is_bit_on(v, 7);
        };
    }

    return ret;
}

bool LSM6DSL::get_id(uint8_t* a_p_id) const
{
    assert(nullptr != a_p_id);

    uint8_t v = 0;
    bool ret  = this->receive(0xFu, &v, sizeof(v));
    if (true == ret)
    {
        (*a_p_id) = v;
    }

    return ret;
}

bool LSM6DSL::get_data(uint8_t a_register, void* a_p_buffer, uint32_t a_buffer_capacity) const
{
    assert(nullptr != a_p_buffer);
    assert(a_buffer_capacity > 0);

    return this->receive(a_register, reinterpret_cast<uint8_t*>(a_p_buffer), a_buffer_capacity);
}

bool LSM6DSL::is_data_avaliable(Sensor a_sensor, bool* a_p_flag) const
{
    assert(nullptr != a_p_flag);

    uint8_t v = 0;
    bool ret  = this->receive(Registers::status_reg, &v, sizeof(v));

    if (true == ret)
    {
        *(a_p_flag) = is_bit_on(v, static_cast<uint8_t>(a_sensor));
    }

    return ret;
}

bool LSM6DSL::Accelerometer::enable(Fullscale_range a_fullscale_range,
                                    Output_data_rate a_output_data_rate,
                                    Power_mode a_power_mode)
{
    assert(Fullscale_range::unknown != a_fullscale_range);
    assert(Output_data_rate::unknown != a_output_data_rate);

    bool ret = false;

    switch (a_power_mode)
    {
        case Power_mode::high_performance: {
            ret = this->p_owner->transmit(Registers::ctrl6_c, 0x0u);
        }
        break;

        case Power_mode::normal: {
            assert(
                (a_output_data_rate == Output_data_rate::_1_6_Hz || a_output_data_rate <= Output_data_rate::_208_Hz));

            ret = this->p_owner->transmit(Registers::ctrl6_c, 0x8u);
        }
        break;

        case Power_mode::unknown: {
            assert(Power_mode::unknown != a_power_mode);
        }
        break;
    }

    if (true == ret)
    {
        ret = this->p_owner->transmit(
            Registers::ctrl1_xl, static_cast<uint8_t>(a_fullscale_range) | static_cast<uint8_t>(a_output_data_rate));
    }

    if (true == ret)
    {
        this->fullscale_range  = a_fullscale_range;
        this->output_data_rate = a_output_data_rate;
        this->power_mode       = a_power_mode;
    }

    return ret;
}

bool LSM6DSL::Accelerometer::disable()
{
    bool ret = this->p_owner->transmit.function(Registers::ctrl1_xl, 0, this->p_owner->transmit.p_user_data);

    if (true == ret)
    {
        this->fullscale_range  = Fullscale_range::unknown;
        this->output_data_rate = Output_data_rate::unknown;
        this->power_mode       = Power_mode::unknown;
    }

    return ret;
}

bool LSM6DSL::Accelerometer::set_fullscale_range(Fullscale_range a_fullscale_range)
{
    assert(Fullscale_range::unknown != a_fullscale_range);

    bool ret = this->p_owner->transmit(
        Registers::ctrl1_xl, static_cast<uint8_t>(a_fullscale_range) | static_cast<uint8_t>(this->output_data_rate));

    if (true == ret)
    {
        this->fullscale_range = a_fullscale_range;
    }

    return ret;
}

bool LSM6DSL::Accelerometer::set_output_data_rate(Output_data_rate a_output_data_rate)
{
    assert(Output_data_rate::unknown != a_output_data_rate);
    assert((Power_mode::normal == this->power_mode && a_output_data_rate <= Output_data_rate::_208_Hz) ||
           Power_mode::high_performance == this->power_mode);

    bool ret = this->p_owner->transmit(
        Registers::ctrl1_xl, static_cast<uint8_t>(this->fullscale_range) | static_cast<uint8_t>(a_output_data_rate));

    if (true == ret)
    {
        this->output_data_rate = a_output_data_rate;
    }

    return ret;
}

bool LSM6DSL::Accelerometer::set_power_mode(Power_mode a_power_mode)
{
    bool ret = this->p_owner->transmit(Registers::ctrl1_xl, 0x0u);

    if (true == ret)
    {
        switch (a_power_mode)
        {
            case Power_mode::high_performance: {
                ret = this->p_owner->transmit(Registers::ctrl6_c, 0x0u);
            }
            break;

            case Power_mode::normal: {
                assert((this->output_data_rate == Output_data_rate::_1_6_Hz ||
                        this->output_data_rate <= Output_data_rate::_208_Hz));

                ret = this->p_owner->transmit(Registers::ctrl6_c, 0x16u);
            }
            break;

            case Power_mode::unknown: {
                assert(Power_mode::unknown != a_power_mode);
            }
            break;
        }
    }

    if (true == ret)
    {
        ret = this->p_owner->transmit(Registers::ctrl1_xl,
                                      static_cast<uint8_t>(this->fullscale_range) |
                                          static_cast<uint8_t>(this->output_data_rate));
    }

    return ret;
}

bool LSM6DSL::Accelerometer::get_axis_raw_polling(Axis<int16_t>* a_p_axis) const
{
    assert(nullptr != a_p_axis);

    bool ret        = true;
    bool data_ready = false;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::accelerometer, &data_ready);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_xl, a_p_axis, sizeof(*a_p_axis));
    }

    return ret && data_ready;
}

bool LSM6DSL::Accelerometer::get_axis_scaled_polling(Axis<float>* a_p_axis) const
{
    assert(nullptr != a_p_axis);

    bool ret                = true;
    const float sensitivity = asl[static_cast<uint32_t>(this->fullscale_range) >> 0x1u];
    int16_t axis[3]         = { 0, 0, 0 };

    bool data_ready = false;
    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::accelerometer, &data_ready);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_xl, axis, sizeof(axis));

        if (true == ret)
        {
            a_p_axis->x = sensitivity * static_cast<float>(axis[0]);
            a_p_axis->y = sensitivity * static_cast<float>(axis[1]);
            a_p_axis->z = sensitivity * static_cast<float>(axis[2]);
        }
    }

    return ret && data_ready;
}

bool LSM6DSL::Accelerometer::get_axis_raw_polling(Axis<int16_t>* a_p_axis, time::tick a_timeout) const
{
    assert(nullptr != a_p_axis);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    bool ret        = true;
    bool data_ready = false;
    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::accelerometer, &data_ready) &&
              a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret)
    {
        ret = this->p_owner->get_data(Registers::outx_l_xl, a_p_axis, sizeof(*a_p_axis));
    }

    return ret && data_ready;
}

bool LSM6DSL::Accelerometer::get_axis_scaled_polling(Axis<float>* a_p_axis, time::tick a_timeout) const
{
    assert(nullptr != a_p_axis);
    assert(a_timeout > 0);

    assert(Fullscale_range::unknown != this->fullscale_range);

    time::tick start = counter::get();

    bool ret                = true;
    const float sensitivity = asl[static_cast<uint32_t>(this->fullscale_range)];
    int16_t axis[3]         = { 0, 0, 0 };

    bool data_ready = false;
    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::accelerometer, &data_ready) &&
              a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_xl, axis, sizeof(axis));

        if (true == ret)
        {
            a_p_axis->x = sensitivity * static_cast<float>(axis[0]);
            a_p_axis->y = sensitivity * static_cast<float>(axis[1]);
            a_p_axis->z = sensitivity * static_cast<float>(axis[2]);
        }
    }

    return ret && data_ready;
}

bool LSM6DSL::Accelerometer::is_enabled(bool* a_p_flag) const
{
    assert(nullptr != a_p_flag);

    uint8_t v = 0;
    bool ret  = this->p_owner->receive(Registers::ctrl1_xl, &v, sizeof(v));

    if (true == ret)
    {
        *a_p_flag = 0 != get_flag(v, 0xF0u);
    }

    return ret;
}

bool LSM6DSL::Gyroscope::enable(Fullscale_range a_fullscale_range,
                                Output_data_rate a_output_data_rate,
                                Power_mode a_power_mode)
{
    assert(Fullscale_range::unknown != a_fullscale_range);
    assert(Output_data_rate::unknown != a_output_data_rate);

    bool ret = false;

    switch (a_power_mode)
    {
        case Power_mode::high_performance: {
            ret = this->p_owner->transmit(Registers::ctrl7_g, 0x0u);
        }
        break;

        case Power_mode::normal: {
            assert(a_output_data_rate <= Output_data_rate::_208_Hz);
            ret = this->p_owner->transmit(Registers::ctrl7_g, 0x80u);
        }
        break;

        case Power_mode::unknown: {
            assert(Power_mode::unknown != a_power_mode);
        }
        break;
    }

    if (true == ret)
    {
        ret = this->p_owner->transmit(
            Registers::ctrl2_g, static_cast<uint8_t>(a_fullscale_range) | static_cast<uint8_t>(a_output_data_rate));
    }

    if (true == ret)
    {
        this->fullscale_range  = a_fullscale_range;
        this->output_data_rate = a_output_data_rate;
        this->power_mode       = a_power_mode;
    }

    return ret;
}

bool LSM6DSL::Gyroscope::disable()
{
    bool ret = this->p_owner->transmit(Registers::ctrl2_g, 0);

    if (true == ret)
    {
        this->fullscale_range  = Fullscale_range::unknown;
        this->output_data_rate = Output_data_rate::unknown;
        this->power_mode       = Power_mode::unknown;
    }

    return ret;
}

bool LSM6DSL::Gyroscope::set_fullscale_range(Fullscale_range a_fullscale_range)
{
    assert(Fullscale_range::unknown != a_fullscale_range);

    bool ret = false;

    if (Fullscale_range::_125_dps != a_fullscale_range)
    {
        ret = this->p_owner->transmit(
            Registers::ctrl2_g, static_cast<uint8_t>(a_fullscale_range) | static_cast<uint8_t>(this->output_data_rate));
    }
    else
    {
        ret = this->p_owner->transmit(Registers::ctrl2_g, 0x1u | static_cast<uint8_t>(this->output_data_rate));
    }

    if (true == ret)
    {
        this->fullscale_range = a_fullscale_range;
    }

    return ret;
}

bool LSM6DSL::Gyroscope::set_output_data_rate(Output_data_rate a_output_data_rate)
{
    assert(Output_data_rate::unknown != a_output_data_rate);
    assert((Power_mode::normal == this->power_mode && a_output_data_rate <= Output_data_rate::_208_Hz) ||
           Power_mode::high_performance == this->power_mode);

    bool ret = this->p_owner->transmit(
        Registers::ctrl2_g, static_cast<uint8_t>(this->fullscale_range) | static_cast<uint8_t>(a_output_data_rate));

    if (true == ret)
    {
        this->output_data_rate = a_output_data_rate;
    }

    return ret;
}

bool LSM6DSL::Gyroscope::set_power_mode(Power_mode a_power_mode)
{
    bool ret = this->p_owner->transmit(Registers::ctrl2_g, 0);

    if (true == ret)
    {
        switch (a_power_mode)
        {
            case Power_mode::high_performance: {
                ret = this->p_owner->transmit(Registers::ctrl7_g, 0x0u);
            }
            break;

            case Power_mode::normal: {
                assert(this->output_data_rate <= Output_data_rate::_208_Hz);
                ret = this->p_owner->transmit(Registers::ctrl7_g, 0x80u);
            }
            break;

            case Power_mode::unknown: {
                assert(Power_mode::unknown != a_power_mode);
            }
            break;
        }
    }

    if (true == ret)
    {
        ret = this->p_owner->transmit(Registers::ctrl2_g,
                                      static_cast<uint8_t>(this->fullscale_range) |
                                          static_cast<uint8_t>(this->output_data_rate));
    }

    if (true == ret)
    {
        this->power_mode = a_power_mode;
    }

    return ret;
}

bool LSM6DSL::Gyroscope::get_axis_raw_polling(Axis<int16_t>* a_p_axis)
{
    assert(nullptr != a_p_axis);

    bool ret        = true;
    bool data_ready = false;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::gyroscope, &data_ready);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_g, a_p_axis, sizeof(*a_p_axis));
    }

    return ret && data_ready;
}

bool LSM6DSL::Gyroscope::get_axis_scaled_polling(Axis<float>* a_p_axis)
{
    assert(nullptr != a_p_axis);

    assert(Fullscale_range::unknown != this->fullscale_range);

    bool ret          = false;
    float sensitivity = gsl[static_cast<uint32_t>(this->fullscale_range) >> 0x2u];
    int16_t axis[3]   = { 0, 0, 0 };

    bool data_ready = false;
    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::gyroscope, &data_ready);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_g, a_p_axis, sizeof(*a_p_axis));

        if (true == ret)
        {
            a_p_axis->x = sensitivity * static_cast<float>(axis[0]);
            a_p_axis->y = sensitivity * static_cast<float>(axis[1]);
            a_p_axis->z = sensitivity * static_cast<float>(axis[2]);
        }
    }

    return ret && data_ready;
}

bool LSM6DSL::Gyroscope::get_axis_raw_polling(Axis<int16_t>* a_p_axis, time::tick a_timeout)
{
    assert(nullptr != a_p_axis);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    bool ret        = true;
    bool data_ready = false;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::gyroscope, &data_ready) &&
              a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_g, a_p_axis, sizeof(*a_p_axis));
    }

    return ret && data_ready;
}
bool LSM6DSL::Gyroscope::get_axis_scaled_polling(Axis<float>* a_p_axis, time::tick a_timeout)
{
    assert(nullptr != a_p_axis);

    assert(Fullscale_range::unknown != this->fullscale_range);

    time::tick start = counter::get();

    bool ret          = false;
    float sensitivity = gsl[static_cast<uint32_t>(this->fullscale_range) >> 0x2u];
    int16_t axis[3]   = { 0, 0, 0 };

    bool data_ready = false;
    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::gyroscope, &data_ready) &&
              a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::outx_l_g, a_p_axis, sizeof(*a_p_axis));

        if (true == ret)
        {
            a_p_axis->x = sensitivity * static_cast<float>(axis[0]);
            a_p_axis->y = sensitivity * static_cast<float>(axis[1]);
            a_p_axis->z = sensitivity * static_cast<float>(axis[2]);
        }
    }

    return ret && data_ready;
}

bool LSM6DSL::Gyroscope::is_enabled(bool* a_p_flag) const
{
    uint8_t v = 0;
    bool ret  = this->p_owner->receive(Registers::ctrl2_g, &v, sizeof(v));

    if (true == ret)
    {
        *a_p_flag = 0 != get_flag(v, 0xF0u);
    }

    return ret;
}

bool LSM6DSL::Temperature::get_data_raw_polling(int16_t* a_p_temperature) const
{
    assert(nullptr != a_p_temperature);

    bool ret        = true;
    bool data_ready = false;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::temperature, &data_ready);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::out_temp_l, a_p_temperature, sizeof(*a_p_temperature));
    }

    return ret && data_ready;
}

bool LSM6DSL::Temperature::get_data_scaled_polling(float* a_p_temperature) const
{
    assert(nullptr != a_p_temperature);

    bool ret        = true;
    bool data_ready = false;
    int16_t sample  = 0;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::temperature, &data_ready);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->receive(Registers::out_temp_l, reinterpret_cast<uint8_t*>(&(sample)), sizeof(sample));

        if (true == sample)
        {
            (*a_p_temperature) = (static_cast<float>(sample) / 256.0f) + 25.0f;
        }
    }

    return ret && data_ready;
}

bool LSM6DSL::Temperature::get_data_raw_polling(int16_t* a_p_temperature, time::tick a_timeout) const
{
    assert(nullptr != a_p_temperature);

    time::tick start = counter::get();

    bool ret        = true;
    bool data_ready = false;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::temperature, &data_ready) &&
              a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->get_data(Registers::out_temp_l, a_p_temperature, sizeof(*a_p_temperature));
    }

    return ret && data_ready;
}

bool LSM6DSL::Temperature::get_data_scaled_polling(float* a_p_temperature, time::tick a_timeout) const
{
    assert(nullptr != a_p_temperature);

    time::tick start = counter::get();

    bool ret        = true;
    bool data_ready = false;
    int16_t sample  = 0;

    while (true == ret && false == data_ready)
    {
        ret = this->p_owner->is_data_avaliable(Sensor::temperature, &data_ready) &&
              a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == data_ready)
    {
        ret = this->p_owner->receive(Registers::out_temp_l, reinterpret_cast<uint8_t*>(&(sample)), sizeof(sample));

        if (true == sample)
        {
            (*a_p_temperature) = (static_cast<float>(sample) / 256.0f) + 25.0f;
        }
    }

    return ret && data_ready;
}

bool LSM6DSL::Fifo::enable(Output_data_rate a_output_data_rate, uint32_t a_pattern_count)
{
    assert(Output_data_rate::unknown != a_output_data_rate);
    assert(0 != a_pattern_count);

    uint32_t gyr_odr_Hz = this->output_data_rate_to_Hz(this->p_owner->p_gyroscope->get_output_data_rate());
    uint32_t acc_odr_Hz = this->output_data_rate_to_Hz(this->p_owner->p_accelerometer->get_output_data_rate());

    uint32_t max_odr_Hz = std::max(acc_odr_Hz, gyr_odr_Hz);
    uint32_t min_odr_Hz = std::min(acc_odr_Hz > 0 ? acc_odr_Hz : 0xFFu, gyr_odr_Hz > 0 ? gyr_odr_Hz : 0xFFu);

    this->pattern.accelerometer = acc_odr_Hz / min_odr_Hz;
    this->pattern.gyroscope     = gyr_odr_Hz / min_odr_Hz;
    this->pattern_count         = a_pattern_count;

    uint32_t watermark = (this->pattern.accelerometer + this->pattern.gyroscope) * 3 * this->pattern_count;

    bool ret = this->p_owner->transmit(Registers::fifo_ctrl1, watermark & 0xFFu) &&
               this->p_owner->transmit(Registers::fifo_ctrl2, (watermark & 0x700u) >> 8u);

    if (true == ret)
    {
        ret = this->p_owner->transmit(Registers::fifo_ctrl5, 0x80u);
    }

    if (true == ret)
    {
        ret = this->p_owner->transmit(Registers::fifo_ctrl3,
                                      this->decimation_to_ctrl3_register(acc_odr_Hz > 0 ? max_odr_Hz / acc_odr_Hz : 0,
                                                                         gyr_odr_Hz > 0 ? max_odr_Hz / gyr_odr_Hz : 0));
    }

    if (true == ret)
    {
        ret = this->p_owner->transmit(Registers::fifo_ctrl5, static_cast<uint8_t>(a_output_data_rate) | 0x1u);
    }

    if (true == ret)
    {
        this->output_data_rate = a_output_data_rate;
    }

    return ret;
}

bool LSM6DSL::Fifo::disable()
{
    bool ret = this->p_owner->transmit.function(Registers::fifo_ctrl5, 0x0u, this->p_owner->transmit.p_user_data);

    if (true == ret)
    {
        this->pattern.accelerometer = 0;
        this->pattern.gyroscope     = 0;
        this->pattern_count         = 0;

        this->output_data_rate = Output_data_rate::unknown;
    }

    return ret;
}

bool LSM6DSL::Fifo::set_output_data_rate(Output_data_rate a_output_data_rate)
{
    assert(Output_data_rate::unknown != a_output_data_rate);

    bool ret = this->p_owner->transmit(Registers::fifo_ctrl5, static_cast<uint8_t>(a_output_data_rate) | 0x1u);

    if (true == ret)
    {
        this->output_data_rate = a_output_data_rate;
    }

    return ret;
}
bool LSM6DSL::Fifo::get_data_raw_polling(Axis<int16_t>* a_p_gyroscope_data,
                                         uint32_t a_gyroscope_data_buffer_capacity,
                                         Axis<int16_t>* a_p_accelerometer_data,
                                         uint32_t a_accelerometer_data_buffer_capacity)
{
    assert((nullptr != a_p_gyroscope_data && a_gyroscope_data_buffer_capacity > 0) || nullptr == a_p_gyroscope_data);

    assert((nullptr != a_p_accelerometer_data && a_accelerometer_data_buffer_capacity > 0) ||
           nullptr == a_p_accelerometer_data);

    bool ret  = true;
    bool full = false;

    while (true == ret && false == full)
    {
        ret = this->is_full(&full);
    }

    if (true == ret && true == full)
    {
        int16_t sample[3] = { 0, 0, 0 };

        for (uint32_t i = 0; i < this->pattern_count && true == ret; i++)
        {
            uint32_t accelerometer_samples_count = 0;
            uint32_t gyroscope_samples_count     = 0;

            for (uint32_t j = 0; j < this->pattern.accelerometer + this->pattern.gyroscope; j++)
            {
                ret = this->p_owner->get_data(Registers::fifo_data_out_l, sample, sizeof(sample));

                if (gyroscope_samples_count < this->pattern.gyroscope &&
                    i + gyroscope_samples_count < a_gyroscope_data_buffer_capacity && true == ret)
                {
                    a_p_gyroscope_data[i + gyroscope_samples_count].x = sample[0];
                    a_p_gyroscope_data[i + gyroscope_samples_count].y = sample[1];
                    a_p_gyroscope_data[i + gyroscope_samples_count].z = sample[2];

                    gyroscope_samples_count++;
                    continue;
                }

                if (accelerometer_samples_count < this->pattern.accelerometer &&
                    i + accelerometer_samples_count < a_accelerometer_data_buffer_capacity && true == ret)
                {
                    a_p_accelerometer_data[i + accelerometer_samples_count].x = sample[0];
                    a_p_accelerometer_data[i + accelerometer_samples_count].y = sample[1];
                    a_p_accelerometer_data[i + accelerometer_samples_count].z = sample[2];

                    accelerometer_samples_count++;
                    continue;
                }
            }
        }

        if (true == ret)
        {
            ret = this->p_owner->transmit(Registers::fifo_ctrl5, 0x0u);

            if (true == ret)
            {
                ret =
                    this->p_owner->transmit(Registers::fifo_ctrl5, static_cast<uint8_t>(this->output_data_rate) | 0x1u);
            }
        }
    }

    return ret && full;
}
bool LSM6DSL::Fifo::get_data_scaled_polling(Axis<float>* a_p_gyroscope_data,
                                            uint32_t a_gyroscope_data_buffer_capacity,
                                            Axis<float>* a_p_accelerometer_data,
                                            uint32_t a_accelerometer_data_buffer_capacity)
{
    assert((nullptr != a_p_gyroscope_data && a_gyroscope_data_buffer_capacity > 0) || nullptr == a_p_gyroscope_data);

    assert((nullptr != a_p_accelerometer_data && a_accelerometer_data_buffer_capacity > 0) ||
           nullptr == a_p_accelerometer_data);

    bool ret  = true;
    bool full = false;

    while (true == ret && false == full)
    {
        ret = this->is_full(&full);
    }

    if (true == ret && true == full)
    {
        const float gs = Gyroscope::Fullscale_range::unknown != this->p_owner->p_gyroscope->get_fullscale_range() ?
                             gsl[static_cast<uint32_t>(this->p_owner->p_gyroscope->get_fullscale_range()) >> 0x1u] :
                             0.0f;
        const float as =
            Accelerometer::Fullscale_range::unknown != this->p_owner->p_accelerometer->get_fullscale_range() ?
                asl[static_cast<uint32_t>(this->p_owner->p_accelerometer->get_fullscale_range()) >> 0x2u] :
                0.0f;

        int16_t sample[3] = { 0, 0, 0 };

        for (uint32_t i = 0; i < this->pattern_count && true == ret; i++)
        {
            uint32_t accelerometer_samples_count = 0;
            uint32_t gyroscope_samples_count     = 0;

            for (uint32_t j = 0; j < this->pattern.accelerometer + this->pattern.gyroscope; j++)
            {
                ret = this->p_owner->get_data(Registers::fifo_data_out_l, sample, sizeof(sample));

                if (gyroscope_samples_count < this->pattern.gyroscope &&
                    i + gyroscope_samples_count < a_gyroscope_data_buffer_capacity && true == ret)
                {
                    a_p_gyroscope_data[i + gyroscope_samples_count].x = gs * static_cast<float>(sample[0]);
                    a_p_gyroscope_data[i + gyroscope_samples_count].y = gs * static_cast<float>(sample[1]);
                    a_p_gyroscope_data[i + gyroscope_samples_count].z = gs * static_cast<float>(sample[2]);

                    gyroscope_samples_count++;
                    continue;
                }

                if (accelerometer_samples_count < this->pattern.accelerometer &&
                    i + accelerometer_samples_count < a_accelerometer_data_buffer_capacity && true == ret)
                {
                    a_p_accelerometer_data[i + accelerometer_samples_count].x = as * static_cast<float>(sample[0]);
                    a_p_accelerometer_data[i + accelerometer_samples_count].y = as * static_cast<float>(sample[1]);
                    a_p_accelerometer_data[i + accelerometer_samples_count].z = as * static_cast<float>(sample[2]);

                    accelerometer_samples_count++;
                    continue;
                }
            }
        }

        if (true == ret)
        {
            ret = this->p_owner->transmit(Registers::fifo_ctrl5, 0x0u);

            if (true == ret)
            {
                ret =
                    this->p_owner->transmit(Registers::fifo_ctrl5, static_cast<uint8_t>(this->output_data_rate) | 0x1u);
            }
        }
    }

    return ret && full;
}

bool LSM6DSL::Fifo::get_data_raw_polling(Axis<int16_t>* a_p_gyroscope_data,
                                         uint32_t a_gyroscope_data_buffer_capacity,
                                         Axis<int16_t>* a_p_accelerometer_data,
                                         uint32_t a_accelerometer_data_buffer_capacity,
                                         time::tick a_timeout)
{
    assert((nullptr != a_p_gyroscope_data && a_gyroscope_data_buffer_capacity > 0) || nullptr == a_p_gyroscope_data);

    assert((nullptr != a_p_accelerometer_data && a_accelerometer_data_buffer_capacity > 0) ||
           nullptr == a_p_accelerometer_data);

    time::tick start = counter::get();

    bool ret  = true;
    bool full = false;

    while (true == ret && false == full)
    {
        ret = this->is_full(&full) && a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == full)
    {
        int16_t sample[3] = { 0, 0, 0 };

        for (uint32_t i = 0; i < this->pattern_count && true == ret; i++)
        {
            uint32_t accelerometer_samples_count = 0;
            uint32_t gyroscope_samples_count     = 0;

            for (uint32_t j = 0; j < this->pattern.accelerometer + this->pattern.gyroscope; j++)
            {
                ret = this->p_owner->get_data(Registers::fifo_data_out_l, sample, sizeof(sample));

                if (gyroscope_samples_count < this->pattern.gyroscope &&
                    i + gyroscope_samples_count < a_gyroscope_data_buffer_capacity && true == ret)
                {
                    a_p_gyroscope_data[i + gyroscope_samples_count].x = sample[0];
                    a_p_gyroscope_data[i + gyroscope_samples_count].y = sample[1];
                    a_p_gyroscope_data[i + gyroscope_samples_count].z = sample[2];

                    gyroscope_samples_count++;
                    continue;
                }

                if (accelerometer_samples_count < this->pattern.accelerometer &&
                    i + accelerometer_samples_count < a_accelerometer_data_buffer_capacity && true == ret)
                {
                    a_p_accelerometer_data[i + accelerometer_samples_count].x = sample[0];
                    a_p_accelerometer_data[i + accelerometer_samples_count].y = sample[1];
                    a_p_accelerometer_data[i + accelerometer_samples_count].z = sample[2];

                    accelerometer_samples_count++;
                    continue;
                }
            }
        }

        if (true == ret)
        {
            ret = this->p_owner->transmit(Registers::fifo_ctrl5, 0x0u);

            if (true == ret)
            {
                ret =
                    this->p_owner->transmit(Registers::fifo_ctrl5, static_cast<uint8_t>(this->output_data_rate) | 0x1u);
            }
        }
    }

    return ret && full;
}

bool LSM6DSL::Fifo::get_data_scaled_polling(Axis<float>* a_p_gyroscope_data,
                                            uint32_t a_gyroscope_data_buffer_capacity,
                                            Axis<float>* a_p_accelerometer_data,
                                            uint32_t a_accelerometer_data_buffer_capacity,
                                            time::tick a_timeout)
{
    assert((nullptr != a_p_gyroscope_data && a_gyroscope_data_buffer_capacity > 0) || nullptr == a_p_gyroscope_data);

    assert((nullptr != a_p_accelerometer_data && a_accelerometer_data_buffer_capacity > 0) ||
           nullptr == a_p_accelerometer_data);

    time::tick start = counter::get();

    bool ret  = true;
    bool full = false;

    while (true == ret && false == full)
    {
        ret = this->is_full(&full) && a_timeout >= time::diff(counter::get(), start);
    }

    if (true == ret && true == full)
    {
        const float gs = Gyroscope::Fullscale_range::unknown != this->p_owner->p_gyroscope->get_fullscale_range() ?
                             gsl[static_cast<uint32_t>(this->p_owner->p_gyroscope->get_fullscale_range()) >> 0x1u] :
                             0.0f;
        const float as =
            Accelerometer::Fullscale_range::unknown != this->p_owner->p_accelerometer->get_fullscale_range() ?
                asl[static_cast<uint32_t>(this->p_owner->p_accelerometer->get_fullscale_range()) >> 0x2u] :
                0.0f;

        int16_t sample[3] = { 0, 0, 0 };

        for (uint32_t i = 0; i < this->pattern_count && true == ret; i++)
        {
            uint32_t accelerometer_samples_count = 0;
            uint32_t gyroscope_samples_count     = 0;

            for (uint32_t j = 0; j < this->pattern.accelerometer + this->pattern.gyroscope; j++)
            {
                ret = this->p_owner->get_data(Registers::fifo_data_out_l, sample, sizeof(sample));

                if (gyroscope_samples_count < this->pattern.gyroscope &&
                    i + gyroscope_samples_count < a_gyroscope_data_buffer_capacity && true == ret)
                {
                    a_p_gyroscope_data[i + gyroscope_samples_count].x = gs * static_cast<float>(sample[0]);
                    a_p_gyroscope_data[i + gyroscope_samples_count].y = gs * static_cast<float>(sample[1]);
                    a_p_gyroscope_data[i + gyroscope_samples_count].z = gs * static_cast<float>(sample[2]);

                    gyroscope_samples_count++;
                    continue;
                }

                if (accelerometer_samples_count < this->pattern.accelerometer &&
                    i + accelerometer_samples_count < a_accelerometer_data_buffer_capacity && true == ret)
                {
                    a_p_accelerometer_data[i + accelerometer_samples_count].x = as * static_cast<float>(sample[0]);
                    a_p_accelerometer_data[i + accelerometer_samples_count].y = as * static_cast<float>(sample[1]);
                    a_p_accelerometer_data[i + accelerometer_samples_count].z = as * static_cast<float>(sample[2]);

                    accelerometer_samples_count++;
                    continue;
                }
            }
        }

        if (true == ret)
        {
            ret = this->p_owner->transmit(Registers::fifo_ctrl5, 0x0u);

            if (true == ret)
            {
                ret =
                    this->p_owner->transmit(Registers::fifo_ctrl5, static_cast<uint8_t>(this->output_data_rate) | 0x1u);
            }
        }
    }

    return ret && full;
}

bool LSM6DSL::Fifo::is_enabled(bool* a_p_flag) const
{
    uint8_t v = 0;
    bool ret =
        this->p_owner->receive.function(Registers::fifo_ctrl5, &v, sizeof(v), this->p_owner->receive.p_user_data);

    if (true == ret)
    {
        uint8_t mode = get_flag(v, 0x7u);
        uint8_t odr  = get_flag(v, 0x78u);

        (*a_p_flag) = 0x1u == mode && 0x0u != odr;
    }

    return ret;
}

bool LSM6DSL::Fifo::is_full(bool* a_p_flag) const
{
    assert(nullptr != a_p_flag);

    uint8_t v = 0;
    bool ret  = this->p_owner->receive(Registers::fifo_status2, &v, sizeof(v));

    if (true == ret)
    {
        (*a_p_flag) = is_bit_on(v, 7);
    }

    return ret;
}

uint32_t LSM6DSL::Fifo::output_data_rate_to_Hz(Accelerometer::Output_data_rate a_output_data_rate) const
{
    return output_data_rate_to_Hz_lut[static_cast<uint8_t>(a_output_data_rate) >> 4u];
}

uint32_t LSM6DSL::Fifo::output_data_rate_to_Hz(Gyroscope::Output_data_rate a_output_data_rate) const
{
    return output_data_rate_to_Hz_lut[static_cast<uint8_t>(a_output_data_rate) >> 4u];
}

uint8_t LSM6DSL::Fifo::decimation_to_ctrl3_register(uint32_t a_accelerometer_decimation,
                                                    uint32_t a_gyroscope_decimation) const
{
    uint8_t ret = 0;

    if (a_accelerometer_decimation > 0)
    {
        if (a_accelerometer_decimation <= 4)
        {
            ret |= (a_accelerometer_decimation & 0x7u);
        }
        else
        {
            switch (a_accelerometer_decimation)
            {
                case 8: {
                    ret |= 0x5u;
                }
                break;

                case 16: {
                    ret |= 0x6u;
                }
                break;

                case 32: {
                    ret |= 0x7u;
                }
                break;
            }
        }
    }

    if (a_gyroscope_decimation > 0)
    {
        if (a_accelerometer_decimation <= 4)
        {
            ret |= ((a_accelerometer_decimation << 0x3u) & 0x38u);
        }
        else
        {
            switch (a_accelerometer_decimation)
            {
                case 8: {
                    ret |= 0x28u;
                }
                break;

                case 16: {
                    ret |= 0x30u;
                }
                break;

                case 32: {
                    ret |= 0x38u;
                }
                break;
            }
        }
    }

    return ret;
}

} // namespace devices
} // namespace cml