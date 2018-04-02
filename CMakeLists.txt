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
SET(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -mthumb-interwork -mfloat-abi=soft -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")
SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")


add_definitions(-DSTM32F103xB)

set(STARTUP_SOURCE "startup/startup_stm32f103xb.s")
file(GLOB_RECURSE USER_SOURCES "Src/*.c")
file(GLOB_RECURSE HAL_SOURCES "Drivers/STM32F1xx_HAL_Driver/Src/*.c")

include_directories(Drivers/CMSIS/Device/ST/STM32F1xx/Include)
include_directories(Drivers/CMSIS/Include)
include_directories(Drivers/STM32F1xx_HAL_Driver/Inc)
include_directories(Inc)

add_executable(${PROJECT_NAME}.elf
        ${USER_SOURCES}
        ${HAL_SOURCES}
        ${STARTUP_SOURCE}
        ${FREERTOS_SOURCES})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_NAME}.map")

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        COMMENT "Building ${PROJECT_NAME}.hex \nBuilding ${PROJECT_NAME}.bin")