cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")

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

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T "${CMAKE_CURRENT_SOURCE_DIR}/Hw/stm32g431cbux_flash.ld"
)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/Hw/stm32g431cbux_flash.ld"
)

target_link_libraries(${CMAKE_PROJECT_NAME} m)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating bin/hex"
)
