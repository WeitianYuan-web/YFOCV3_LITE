cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Prefer CMAKE_TOOLCHAIN_FILE (CMake Presets / CLion) over include().
# include() before project() is too late to override a host compiler CLion may inject,
# and double-loads flags when the preset also sets toolchainFile.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "ARM GCC toolchain" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_PROJECT_NAME YFOCV3)

project(${CMAKE_PROJECT_NAME} C ASM)

add_executable(${CMAKE_PROJECT_NAME})

set(HAL_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver")
set(CMSIS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Hw/startup_stm32g431xx.s
    Hw/system_stm32g4xx.c
    Hw/sysmem.c
    Hw/syscalls.c
    Hw/board.c
    Hw/irq.c
    Hw/pwm.c
    Hw/encoder.c
    Hw/can.c
    Hw/debug.c
    Hw/RTT/SEGGER_RTT.c
    Hw/RTT/SEGGER_RTT_printf.c

    ${HAL_DIR}/Src/stm32g4xx_hal.c
    ${HAL_DIR}/Src/stm32g4xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_dma.c
    ${HAL_DIR}/Src/stm32g4xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_exti.c
    ${HAL_DIR}/Src/stm32g4xx_hal_fdcan.c
    ${HAL_DIR}/Src/stm32g4xx_hal_flash.c
    ${HAL_DIR}/Src/stm32g4xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_flash_ramfunc.c
    ${HAL_DIR}/Src/stm32g4xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32g4xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32g4xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32g4xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_spi.c
    ${HAL_DIR}/Src/stm32g4xx_hal_spi_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_tim.c
    ${HAL_DIR}/Src/stm32g4xx_hal_tim_ex.c

    Foc/foc_math.c
    Foc/foc_svpwm.c
    Foc/foc_encoder.c

    App/main.c
    App/servo.c
    App/cali.c
    App/cali_nv.c
    App/comm.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Foc
    Hw
    Hw/RTT
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32G4xx/Include
    ${CMSIS_DIR}/Include
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32G431xx
    $<$<CONFIG:Debug>:DEBUG>
)

# Forward slashes: GNU ld on Windows rejects mixed/backslash -T paths.
file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Hw/stm32g431cbux_flash.ld" YFOC_LINKER_SCRIPT)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T "${YFOC_LINKER_SCRIPT}"
    "LINKER:-Map=${CMAKE_PROJECT_NAME}.map"
)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINK_DEPENDS "${YFOC_LINKER_SCRIPT}"
)

target_link_libraries(${CMAKE_PROJECT_NAME} m)

if(NOT CMAKE_OBJCOPY)
    message(FATAL_ERROR "CMAKE_OBJCOPY is not set (arm-none-eabi-objcopy not found)")
endif()
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND "${CMAKE_OBJCOPY}" -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating bin/hex"
)
