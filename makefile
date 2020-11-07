ifndef NOSILENT
.SILENT:
endif

CML_CPP_FILES := lib/cml/common/*.cpp                  \
				 lib/cml/debug/*.cpp                   \
				 lib/cml/utils/*.cpp                   \
				 lib/devices/LSM6DSL/*.cpp             \
				 lib/soc/*.cpp                         \
				 lib/soc/stm32l011xx/*.cpp             \
				 lib/soc/stm32l011xx/peripherals/*.cpp \
				 lib/soc/stm32l011xx/system/*.cpp      \
				 lib/soc/stm32l452xx/*.cpp             \
				 lib/soc/stm32l452xx/peripherals/*.cpp \
				 lib/soc/stm32l452xx/system/*.cpp      \

CML_HPP_FILES := lib/cml/*.hpp                         \
				 lib/cml/common/*.hpp                  \
				 lib/cml/debug/*.hpp                   \
				 lib/cml/hal/*.hpp                     \
				 lib/cml/hal/peripherals/*.hpp         \
				 lib/cml/hal/system/*.hpp              \
				 lib/cml/utils/*.hpp                   \
				 lib/devices/LSM6DSL/*.hpp             \
				 lib/soc/*.hpp                         \
				 lib/soc/stm32l011xx/*.hpp             \
				 lib/soc/stm32l011xx/peripherals/*.hpp \
				 lib/soc/stm32l011xx/system/*.hpp      \
				 lib/soc/stm32l452xx/*.hpp             \
				 lib/soc/stm32l452xx/peripherals/*.hpp \
				 lib/soc/stm32l452xx/system/*.hpp      \

STM32L011XX_SAMPLES := samples/stm32l011xx/*/*.cpp
STM32L452XX_SAMPLES := samples/stm32l452xx/*/*.cpp

.PHONY: format

format:
	clang-format -style=file -i $(CML_CPP_FILES) $(CML_HPP_FILES) $(STM32L011XX_SAMPLES) $(STM32L452XX_SAMPLES)