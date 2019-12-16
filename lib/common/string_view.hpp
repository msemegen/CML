#pragma once

/*
    Name: string_view.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>
#include <common/numeric_traits.hpp>

namespace cml {
namespace common {

class c_string_view
{
public:

     c_string_view(char* a_p_buffer, uint32 a_capacity)
         : p_buffer(a_p_buffer)
         , capacity(a_capacity)
         , length(0)
     {
         for (decltype(a_capacity) i = 0; i < a_capacity; i++)
         {
             this->p_buffer[i] = 0;
         }
     }

     c_string_view(char* a_p_buffer, uint32 a_capacity, const char* a_p_init_data)
         : c_string_view(a_p_buffer, a_capacity)
     {
         _assert(this->capacity > 0);

         uint32 init_data_length = cstring_length(a_p_init_data, c_numeric_traits<uint32>::get_max());
         for (uint32 i = 0; i < init_data_length && this->capacity - 1; i++)
         {
             this->p_buffer[i] = a_p_init_data[i];
         }
     }

     c_string_view()                     = delete;
     c_string_view(c_string_view&&)      = default;
     c_string_view(const c_string_view&) = default;
     ~c_string_view()                    = default;

     c_string_view& operator = (c_string_view&&)      = default;
     c_string_view& operator = (const c_string_view&) = default;

     char operator[](uint32 a_index) const
     {
         _assert(a_index < this->length);
         return this->p_buffer[a_index];
     }

     char& operator[](uint32 a_index)
     {
         _assert(a_index < this->length);
         return this->p_buffer[a_index];
     }

     bool operator == (const char* a_p_other) const
     {
         uint32 other_string_length = cstring_length(a_p_other, c_numeric_traits<uint32>::get_max());
         bool retval = other_string_length == this->length;

         if (true == retval)
         {
             retval = memory_equals(a_p_other, this->p_buffer, this->length);
         }

         return retval;
     }

     bool operator == (const c_string_view& a_other) const
     {
         bool retval = this->length == a_other.get_length();

         if (true == retval)
         {
             retval = memory_equals(a_other.p_buffer, this->p_buffer, this->length);
         }

         return retval;
     }

     bool operator != (const char* a_p_other) const
     {
         return false == (*this == a_p_other);
     }

     bool operator != (const c_string_view& a_other) const
     {
         return false == (*this == a_other);
     }

     void push_back(char a_c)
     {
         _assert(this->capacity > this->length + 1);
         this->p_buffer[this->length++] = a_c;
     }

     void pop_back()
     {
         _assert(this->length > 0);
         this->p_buffer[this->length--] = 0;
     }

     void clear()
     {
         for (decltype(this->length) i = 0; i < this->length; i++)
         {
             this->p_buffer[i] = 0;
         }

         this->length = 0;
     }

     uint32 get_capacity() const
     {
         return this->capacity;
     }

     uint32 get_length() const
     {
         return this->length;
     }

     bool is_full() const
     {
         return this->length + 1 == this->capacity;
     }

     bool is_empty() const
     {
         return 0 == this->length;
     }

     const char* get_cstring() const
     {
         return this->p_buffer;
     }

 private:

     char* p_buffer;
     uint32 capacity;
     uint32 length;
 };

} // namespace common
} // namespace cml