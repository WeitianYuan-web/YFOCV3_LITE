set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# CMAKE_SYSTEM_* must be set on every toolchain load. Flags and find_* must not.
if(YFOC_ARM_TOOLCHAIN_CONFIGURED)
    return()
endif()
set(YFOC_ARM_TOOLCHAIN_CONFIGURED TRUE)

if(CMAKE_HOST_WIN32)
    set(_YFOC_HOST_EXE ".exe")
else()
    set(_YFOC_HOST_EXE "")
endif()

# ---------------------------------------------------------------------------
# Host Ninja (Windows: often bundled with CLion / CubeCLT, not on PATH)
# Must run here so CMake Presets' Ninja generator can see CMAKE_MAKE_PROGRAM.
# ---------------------------------------------------------------------------
if(NOT CMAKE_MAKE_PROGRAM)
    set(_YFOC_NINJA_DIRS "")
    if(CMAKE_HOST_WIN32)
        file(GLOB _YFOC_NINJA_DIRS
            "C:/Program Files/JetBrains/CLion */bin/ninja/win/x64"
            "C:/Program Files (x86)/JetBrains/CLion */bin/ninja/win/x64"
            "D:/Program Files/JetBrains/CLion */bin/ninja/win/x64"
            "D:/JetBrains/CLion */bin/ninja/win/x64"
            "$ENV{LOCALAPPDATA}/Programs/CLion/bin/ninja/win/x64"
            "C:/ST/STM32CubeCLT*/Ninja/bin"
            "D:/ST/STM32CubeCLT*/Ninja/bin"
            "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeCLT*/Ninja/bin"
        )
        if(_YFOC_NINJA_DIRS)
            list(SORT _YFOC_NINJA_DIRS COMPARE NATURAL ORDER DESCENDING)
        endif()
    endif()

    find_program(CMAKE_MAKE_PROGRAM
        NAMES ninja ninja.exe
        PATHS ${_YFOC_NINJA_DIRS}
        DOC "Ninja build tool"
    )
endif()

# ---------------------------------------------------------------------------
# ARM GCC: PATH first, then well-known Windows install locations.
# Override with -DARM_NONE_EABI_TOOLCHAIN_PATH=<bin dir> or env of the same name.
# ---------------------------------------------------------------------------
set(_YFOC_ARM_HINTS "")
if(ARM_NONE_EABI_TOOLCHAIN_PATH)
    list(APPEND _YFOC_ARM_HINTS "${ARM_NONE_EABI_TOOLCHAIN_PATH}")
endif()
if(DEFINED ENV{ARM_NONE_EABI_TOOLCHAIN_PATH})
    list(APPEND _YFOC_ARM_HINTS "$ENV{ARM_NONE_EABI_TOOLCHAIN_PATH}")
endif()

if(CMAKE_HOST_WIN32)
    file(GLOB _YFOC_ARM_GLOBS
        "C:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
        "D:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
        "C:/Program Files/STMicroelectronics/STM32CubeIDE*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
        "C:/ST/STM32CubeCLT*/GNU-tools-for-STM32/bin"
        "D:/ST/STM32CubeCLT*/GNU-tools-for-STM32/bin"
        "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeCLT*/GNU-tools-for-STM32/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
        "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
        "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin"
        "C:/Program Files (x86)/GNU Tools ARM Embedded/*/bin"
    )
    if(_YFOC_ARM_GLOBS)
        list(SORT _YFOC_ARM_GLOBS COMPARE NATURAL ORDER DESCENDING)
        list(APPEND _YFOC_ARM_HINTS ${_YFOC_ARM_GLOBS})
    endif()
endif()

find_program(YFOC_ARM_GCC
    NAMES arm-none-eabi-gcc arm-none-eabi-gcc.exe
    PATHS ${_YFOC_ARM_HINTS}
    DOC "ARM GCC cross compiler"
)

if(NOT YFOC_ARM_GCC)
    message(FATAL_ERROR
        "arm-none-eabi-gcc not found.\n"
        "Install STM32CubeIDE, STM32CubeCLT, or Arm GNU Toolchain, and either:\n"
        "  - add the toolchain bin directory to PATH, or\n"
        "  - set ARM_NONE_EABI_TOOLCHAIN_PATH to that bin directory."
    )
endif()

get_filename_component(YFOC_ARM_TOOLCHAIN_DIR "${YFOC_ARM_GCC}" DIRECTORY)

find_program(YFOC_ARM_GXX
    NAMES arm-none-eabi-g++ arm-none-eabi-g++.exe
    HINTS "${YFOC_ARM_TOOLCHAIN_DIR}"
    NO_DEFAULT_PATH
)
find_program(YFOC_ARM_OBJCOPY
    NAMES arm-none-eabi-objcopy arm-none-eabi-objcopy.exe
    HINTS "${YFOC_ARM_TOOLCHAIN_DIR}"
    NO_DEFAULT_PATH
)
find_program(YFOC_ARM_SIZE
    NAMES arm-none-eabi-size arm-none-eabi-size.exe
    HINTS "${YFOC_ARM_TOOLCHAIN_DIR}"
    NO_DEFAULT_PATH
)

if(NOT YFOC_ARM_GXX)
    set(YFOC_ARM_GXX "${YFOC_ARM_TOOLCHAIN_DIR}/arm-none-eabi-g++${_YFOC_HOST_EXE}")
endif()
if(NOT YFOC_ARM_OBJCOPY)
    set(YFOC_ARM_OBJCOPY "${YFOC_ARM_TOOLCHAIN_DIR}/arm-none-eabi-objcopy${_YFOC_HOST_EXE}")
endif()
if(NOT YFOC_ARM_SIZE)
    set(YFOC_ARM_SIZE "${YFOC_ARM_TOOLCHAIN_DIR}/arm-none-eabi-size${_YFOC_HOST_EXE}")
endif()

set(CMAKE_C_COMPILER   "${YFOC_ARM_GCC}"     CACHE FILEPATH "C compiler" FORCE)
set(CMAKE_ASM_COMPILER "${YFOC_ARM_GCC}"     CACHE FILEPATH "ASM compiler" FORCE)
set(CMAKE_CXX_COMPILER "${YFOC_ARM_GXX}"     CACHE FILEPATH "CXX compiler" FORCE)
set(CMAKE_LINKER       "${YFOC_ARM_GXX}"     CACHE FILEPATH "Linker" FORCE)
set(CMAKE_OBJCOPY      "${YFOC_ARM_OBJCOPY}" CACHE FILEPATH "objcopy" FORCE)
set(CMAKE_SIZE         "${YFOC_ARM_SIZE}"    CACHE FILEPATH "size" FORCE)

message(STATUS "ARM GCC: ${YFOC_ARM_GCC}")
if(CMAKE_MAKE_PROGRAM)
    message(STATUS "Ninja:   ${CMAKE_MAKE_PROGRAM}")
endif()

set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

# Release is -O2. FOC/ISR files add -O3 in CMakeLists (local, not whole firmware).
# MinSizeRel stays -Os if Flash gets tight. Debug stays -Og.
set(CMAKE_C_FLAGS_DEBUG          "-Og -g3"          CACHE STRING "Debug C flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE        "-O2 -g0 -DNDEBUG" CACHE STRING "Release C flags" FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3 -DNDEBUG" CACHE STRING "RelWithDebInfo C flags" FORCE)
set(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG"     CACHE STRING "MinSizeRel C flags" FORCE)

set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_C_FLAGS_DEBUG}"          CACHE STRING "Debug CXX flags" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_C_FLAGS_RELEASE}"        CACHE STRING "Release CXX flags" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}" CACHE STRING "RelWithDebInfo CXX flags" FORCE)
set(CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_C_FLAGS_MINSIZEREL}"     CACHE STRING "MinSizeRel CXX flags" FORCE)

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")
