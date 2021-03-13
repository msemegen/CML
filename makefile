#
#  Name: makefile.mk
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

ifndef NOSILENT
.SILENT:
endif

CML_CPP_FILES := lib/cml/debug/*.cpp                   \
				 lib/soc/*.cpp                         \
				 lib/soc/stm32l011xx/*.cpp             \
				 lib/soc/stm32l011xx/peripherals/*.cpp \
				 lib/soc/stm32l452xx/*.cpp             \
				 lib/soc/stm32l452xx/peripherals/*.cpp

CML_HPP_FILES := lib/cml/*.hpp                         \
				 lib/cml/debug/*.hpp                   \
				 lib/cml/hal/*.hpp                     \
				 lib/cml/hal/peripherals/*.hpp         \
				 lib/cml/utils/*.hpp                   \
				 lib/soc/*.hpp                         \
				 lib/soc/stm32l011xx/*.hpp             \
				 lib/soc/stm32l011xx/peripherals/*.hpp \
				 lib/soc/stm32l452xx/*.hpp             \
				 lib/soc/stm32l452xx/peripherals/*.hpp

STM32L011XX_TEMPLATES := templates/stm32l011xx/*/*.cpp
STM32L452XX_TEMPLATES := templates/stm32l452xx/*/*.cpp

.PHONY: format

format:
	clang-format -style=file -i $(CML_CPP_FILES) $(CML_HPP_FILES) $(STM32L011XX_TEMPLATES) $(STM32L452XX_TEMPLATES)