#pragma once

namespace cml {
namespace hal {
namespace stm32l4xx {

 class c_i2c_master
 {
 public:

     enum class e_periph
     {

     };

 public:

     c_i2c_master();

     void write();
     void read();

     void write_bytes();
     void read_bytes();
 };

} // namespace stm32l4xx
} // namespace cml
} // namespace hal