#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RNG/Polling.hpp>

// soc
#include <soc/system_timer.hpp>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
#define RNG_T ((RNG_TypeDef*)RNG_BASE)

using namespace cml::utils;

bool Polling<RNG>::get_value(std::uint32_t* a_p_value, std::uint32_t a_timeout)
{
    cml_assert(a_timeout > 0);

    std::uint32_t start = system_timer::get();

    bool ret = wait_until::all_bits(&(RNG_T->SR), RNG_SR_DRDY, false, start, a_timeout);

    if (true == ret)
    {
        (*a_p_value) = RNG_T->DR;
    }

    return ret;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif