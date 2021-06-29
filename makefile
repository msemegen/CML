#
#  Name: makefile.mk
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

ifndef NOSILENT
.SILENT:
endif

CML_CPP_FILES := lib/cml/debug/*.cpp                 \
				 lib/soc/*.cpp                       \
				 lib/soc/m4/stm32l4/*.cpp            \
				 lib/soc/m4/stm32l4/peripherals/*.cpp

CML_HPP_FILES := lib/cml/*.hpp                        \
				 lib/cml/debug/*.hpp                  \
				 lib/cml/hal/*.hpp                    \
				 lib/cml/hal/peripherals/*.hpp        \
				 lib/cml/utils/*.hpp                  \
				 lib/soc/*.hpp                        \
				 lib/soc/m4/*.hpp                     \
				 lib/soc/m4/stm32l4/*.hpp             \
				 lib/soc/m4/stm32l4/peripherals/*.hpp

STM32L4_TEMPLATES := templates/stm32l4/*/*/*.cpp

.PHONY: format

format:
	clang-format -style=file -i $(CML_CPP_FILES) $(CML_HPP_FILES) $(STM32L4_TEMPLATES)