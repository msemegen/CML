#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class SPI_base : public cml::Non_copyable
{
};

class SPI_master : public SPI_base
{
};

class SPI_slave : public SPI_base
{
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc
