project(stm32 C ASM)
cmake_minimum_required(VERSION 3.5.0)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(TOOLCHAIN "/Users/zjz/stm32/gcc-arm-none-eabi/bin/")
set(CMAKE_ASM_COMPILER ${TOOLCHAIN}/arm-none-eabi-gcc)
SET(CMAKE_C_COMPILER ${TOOLCHAIN}/arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN}/arm-none-eabi-gcc)
SET(CMAKE_OBJCOPY ${TOOLCHAIN}/arm-none-eabi-objcopy)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)
SET(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -mthumb-interwork -mfloat-abi=soft -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0 -specs=nano.specs -specs=nosys.specs")
SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")

add_definitions(-DSTM32F103xE)

file(GLOB_RECURSE SOURCES "startup/*.*" "Drivers/*.*" "Src/*.*" "freertos/*.*" "freertos/portable/*.c")

include_directories(Inc
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
        freertos/include
        freertos/portable)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")