# CML - Cortex-M Library

CML is an open source library that provides hardware abstraction layer for STM32L4 family microcontrollers covered in [RM0394](https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf). It supports variety of peripherals including DMA and non DMA interfaces. 
C++ 17 ensures full portability across various of modern C++ toolchains. Consistent API design makes CML easy to use and contribute. Compile time checks with conditional compilation guarantees that used API is available for given microcontroller.
In addition library provides several useful tools like asserts with custom print and MCU halt handlers, bit flag operations, us/ms delays etc.

# Supported peripherals:
