#pragma once

/*
 *   Name: Non_constructible.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

namespace cml {
class Non_constructible
{
protected:
    Non_constructible()                         = delete;
    Non_constructible(const Non_constructible&) = delete;
    Non_constructible(Non_constructible&&)      = delete;

    Non_constructible& operator=(const Non_constructible&) = delete;
    Non_constructible& operator=(Non_constructible&&) = delete;
};
} // namespace cml