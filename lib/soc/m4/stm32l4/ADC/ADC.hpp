#pragma once

/*
 *   Name: ADC.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <tuple>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class ADC : private cml::Non_copyable
{
public:
    enum class Resolution : std::uint32_t
    {
        _6_bit  = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
        _8_bit  = ADC_CFGR_RES_1,
        _10_bit = ADC_CFGR_RES_0,
        _12_bit = 0u,
    };

    struct Calibration_data
    {
        std::uint16_t temperature_sensor_data_1  = 0u;
        std::uint16_t temperature_sensor_data_2  = 0u;
        std::uint16_t internal_voltage_reference = 0u;
    };

    ~ADC();

    void enable(Resolution a_resolution);
    bool enable(Resolution a_resolution, std::uint32_t a_timeout);

    void disable();

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const std::uint16_t*>(0x1FFF75A8)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75CA)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75AA)) };
    }

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    operator ADC_TypeDef*()
    {
        return this->p_registers;
    }

    operator const ADC_TypeDef*() const
    {
        return this->p_registers;
    }

private:
    ADC(std::size_t a_idx, ADC_TypeDef* a_p_registers)
        : idx(a_idx)
        , p_registers(a_p_registers)
    {
    }

    const std::uint32_t idx;
    ADC_TypeDef* p_registers;

    template<typename Periph_t, std::size_t id> friend class Factory;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc