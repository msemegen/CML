#pragma once

/*
    Name: Non_copyable.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {

class Non_copyable
{
public:
    Non_copyable(const Non_copyable&) = delete;
    Non_copyable& operator=(const Non_copyable&) = delete;

protected:
    Non_copyable()               = default;
    Non_copyable(Non_copyable&&) = default;
    ~Non_copyable()              = default;

    Non_copyable& operator=(Non_copyable&&) = default;
};

} // namespace cml