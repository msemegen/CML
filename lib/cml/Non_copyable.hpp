#pragma once

/*
 *   Name: Non_copyable.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

namespace cml {
class Non_copyable
{
protected:
    Non_copyable(const Non_copyable&) = delete;

    Non_copyable()               = default;
    Non_copyable(Non_copyable&&) = default;

    Non_copyable& operator=(Non_copyable&&) = default;

protected:
    Non_copyable& operator=(const Non_copyable&) = default;
};
} // namespace cml