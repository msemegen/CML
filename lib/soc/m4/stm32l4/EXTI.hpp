#pragma once

/*
 *   Name: EXTI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/Non_copyable.hpp>

// soc
#include <soc/m4/stm32l4/peripherals/GPIO.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

#ifdef EXTI
#undef EXTI
#endif

template<typename Peripheral_t> class EXTI : private cml::Non_copyable
{
};

template<> class EXTI<peripherals::GPIO> : private cml::Non_copyable
{
public:
    enum class Id
    {
        _0     = EXTI0_IRQn,
        _1     = EXTI1_IRQn,
        _2     = EXTI2_IRQn,
        _3     = EXTI3_IRQn,
        _4     = EXTI4_IRQn,
        _5_9   = EXTI9_5_IRQn,
        _10_15 = EXTI15_10_IRQn
    };

    enum class Trigger_flag : uint32_t
    {
        rising  = 0x1,
        falling = 0x2,
    };

    struct Callback
    {
        using Function = void (*)(uint32_t a_pin, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    EXTI(Id a_id)
        : id(a_id)
    {
    }

    ~EXTI()
    {
        this->disable();
    }

    void enable(const Callback& a_callback, uint32_t a_priority);
    void disable();

    void attach(const peripherals::GPIO& a_port, uint32_t a_pin, Trigger_flag a_trigger);
    void deattach(const peripherals::GPIO& a_port, uint32_t a_pin);

    Id get_id() const
    {
        return this->id;
    }

private:
    Id id;
};

constexpr EXTI<peripherals::GPIO>::Trigger_flag operator|(EXTI<peripherals::GPIO>::Trigger_flag a_f1,
                                                          EXTI<peripherals::GPIO>::Trigger_flag a_f2)
{
    return static_cast<EXTI<peripherals::GPIO>::Trigger_flag>(static_cast<uint32_t>(a_f1) |
                                                              static_cast<uint32_t>(a_f2));
}
constexpr EXTI<peripherals::GPIO>::Trigger_flag operator&(EXTI<peripherals::GPIO>::Trigger_flag a_f1,
                                                          EXTI<peripherals::GPIO>::Trigger_flag a_f2)
{
    return static_cast<EXTI<peripherals::GPIO>::Trigger_flag>(static_cast<uint32_t>(a_f1) &
                                                              static_cast<uint32_t>(a_f2));
}
constexpr EXTI<peripherals::GPIO>::Trigger_flag operator|=(EXTI<peripherals::GPIO>::Trigger_flag& a_f1,
                                                           EXTI<peripherals::GPIO>::Trigger_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc