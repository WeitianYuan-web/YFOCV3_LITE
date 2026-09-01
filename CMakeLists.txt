cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "ARM GCC toolchain" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_PROJECT_NAME YFOCV3)

project(${CMAKE_PROJECT_NAME} C ASM)

set(CMAKE_C_FLAGS_RELEASE        "-O2 -g0 -DNDEBUG" CACHE STRING "Release C flags" FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3 -DNDEBUG" CACHE STRING "RelWithDebInfo C flags" FORCE)

add_executable(${CMAKE_PROJECT_NAME})

set(GD32_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/GD32F30x")
set(GD32_CMSIS "${GD32_ROOT}/CMSIS")
set(GD32_SPL   "${GD32_ROOT}/GD32F30x_standard_peripheral")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Hw/startup_gd32f30x_hd.S
    ${GD32_CMSIS}/GD/GD32F30x/Source/system_gd32f30x.c
    ${GD32_CMSIS}/GD/GD32F30x/Source/syscalls.c
    Hw/board.c
    Hw/irq.c
    Hw/pwm.c
    Hw/encoder.c
    Hw/can.c
    Hw/debug.c
    Hw/adc.c
    Hw/RTT/SEGGER_RTT.c
    Hw/RTT/SEGGER_RTT_printf.c

    ${GD32_SPL}/Source/gd32f30x_adc.c
    ${GD32_SPL}/Source/gd32f30x_can.c
    ${GD32_SPL}/Source/gd32f30x_dbg.c
    ${GD32_SPL}/Source/gd32f30x_fmc.c
    ${GD32_SPL}/Source/gd32f30x_gpio.c
    ${GD32_SPL}/Source/gd32f30x_misc.c
    ${GD32_SPL}/Source/gd32f30x_pmu.c
    ${GD32_SPL}/Source/gd32f30x_rcu.c
    ${GD32_SPL}/Source/gd32f30x_spi.c
    ${GD32_SPL}/Source/gd32f30x_timer.c

    Foc/foc_math.c
    Foc/foc_svpwm.c
    Foc/foc_encoder.c

    App/main.c
    App/servo.c
    App/cali.c
    App/cali_nv.c
    App/comm.c
    App/node.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Foc
    Hw
    Hw/RTT
    ${GD32_CMSIS}
    ${GD32_CMSIS}/GD/GD32F30x/Include
    ${GD32_SPL}/Include
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    GD32F30X_HD
    USE_STDPERIPH_DRIVER
    HXTAL_VALUE=32000000
    $<$<CONFIG:Debug>:DEBUG>
)

set_source_files_properties(
    Foc/foc_math.c
    Foc/foc_svpwm.c
    Foc/foc_encoder.c
    Hw/encoder.c
    Hw/pwm.c
    App/servo.c
    PROPERTIES
        COMPILE_OPTIONS "$<$<CONFIG:Release,RelWithDebInfo>:-O3;-fno-math-errno;-fno-trapping-math>"
)

file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Hw/gd32f30x_flash.ld" YFOC_LINKER_SCRIPT)
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
