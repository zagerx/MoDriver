cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MoDrive)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
)

# 配置 motorlib_cfg.h 的路径（可由用户通过 -DMOTORLIB_CFG_PATH=... 覆盖）
if(NOT DEFINED MOTORLIB_PARAMCONFIG_PATH)
    set(MOTORLIB_PARAMCONFIG_PATH ${CMAKE_SOURCE_DIR}/App)
endif()
get_filename_component(MOTORLIB_PARAMCONFIG_PATH "${MOTORLIB_PARAMCONFIG_PATH}" ABSOLUTE)
message(STATUS "motorlib_cfg.h path: ${MOTORLIB_PARAMCONFIG_PATH}")

# 增加hardware_driver目标
add_subdirectory(Hardware)
add_subdirectory(motorlib)
add_subdirectory(CMSIS-DSP)
add_subdirectory(statemachine)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/main.c
)

# 工程依赖
target_link_libraries(${CMAKE_PROJECT_NAME} 
    hardware_driver
    motor
)




##################### 生成hex ##############################
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                                    ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Creating ${CMAKE_PROJECT_NAME}.hex")


# ==================================================
# 添加 JLink 下载目标（兼容多配置构建）
#cmake --build build/ --target jlink_down
# ==================================================
# MCU 型号（用于 JLink 下载）
set(CHIP "STM32G431CB")

# ==================================================
# 必须指定 chip 型号，否则 JLink 下载无法工作
# ==================================================
if(NOT DEFINED CHIP)
    message(FATAL_ERROR 
        "CHIP is not defined!\n"
        "Please specify the MCU model used by JLink.\n"
        "For example:\n"
        "    set(CHIP \"STM32G431CB\")\n"
    )
endif()

# 寻找 JLink 工具（兼容 Windows / Linux）
if(WIN32)
    find_program(JLINKEXE NAMES JLink.exe)
else()
    find_program(JLINKEXE NAMES JLinkExe)
endif()

if(NOT JLINKEXE)
    message(WARNING "JLink not found, JLink-based download target will NOT be available")
else()
    message(STATUS "Found JLink: ${JLINKEXE}")

    # 生成 jlink.jlink 脚本
    set(JLINK_SCRIPT ${CMAKE_BINARY_DIR}/jlink.jlink)

    add_custom_command(
        OUTPUT ${JLINK_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E echo "h" > ${JLINK_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E echo "loadfile $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}${OUTPUT_SUFFIX}.hex" >> ${JLINK_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E echo "r" >> ${JLINK_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E echo "q" >> ${JLINK_SCRIPT}
        COMMENT "Generating JLink download script"
    )

    # jlink_down 目标（CMake 多配置构建下可用）
    add_custom_target(jlink_down
        COMMAND ${JLINKEXE}
                -device ${CHIP}
                -Speed 4000
                -IF SWD
                -CommanderScript ${JLINK_SCRIPT}
        DEPENDS ${CMAKE_PROJECT_NAME} ${JLINK_SCRIPT}
        COMMENT "Downloading ${CMAKE_PROJECT_NAME}${OUTPUT_SUFFIX}.hex using JLink"
        USES_TERMINAL
    )
endif()


