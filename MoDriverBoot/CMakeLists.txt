cmake_minimum_required(VERSION 3.20)

#---------------------------------------------------------------------------
# Cross-compilation toolchain (must be set BEFORE project())
#---------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

project(openblt_stm32g473 C ASM)

#---------------------------------------------------------------------------
# Output directory (all build artifacts go to MoDriverBoot/build/)
#---------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

# Export compile_commands.json for clangd / IDE auto-completion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Copy compile_commands.json to the project root so clangd can find it easily
add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
    DEPENDS openblt_stm32g473
    COMMENT "Copying compile_commands.json to project root"
)

#---------------------------------------------------------------------------
# MCU flags (match the original makefile exactly)
#---------------------------------------------------------------------------
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

add_compile_options(
    ${MCU_FLAGS}
    -fno-strict-aliasing
    -fdata-sections
    -ffunction-sections
    -Wall
    -g3
    -Og
)

add_compile_definitions(
    USE_FULL_LL_DRIVER
    USE_HAL_DRIVER
    STM32G473xx
)

# Macros that contain special characters – pass them as raw -D flags.
add_compile_options(
    -D__weak=__attribute__\(\(weak\)\)
    -D__packed=__attribute__\(\(__packed__\)\)
)

#---------------------------------------------------------------------------
# Linker flags
#---------------------------------------------------------------------------
add_link_options(
    ${MCU_FLAGS}
    -Og
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32G473RCTx_FLASH.ld
    -specs=nano.specs
    -Wl,--gc-sections
)

#---------------------------------------------------------------------------
# Include directories from lib/
#---------------------------------------------------------------------------
set(LIB_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Inc/Legacy
)

#---------------------------------------------------------------------------
# HAL/LL sources
#---------------------------------------------------------------------------
set(HAL_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/system_stm32g4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c
)

#---------------------------------------------------------------------------
# Application sources
#---------------------------------------------------------------------------
set(APP_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/hooks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/led.c
)

#---------------------------------------------------------------------------
# Startup assembly
#---------------------------------------------------------------------------
set(STARTUP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32g474xx.s
)

#---------------------------------------------------------------------------
# OpenBLT core / port (interface targets defined in Source/CMakeLists.txt)
#---------------------------------------------------------------------------
add_subdirectory(Source)

#---------------------------------------------------------------------------
# Executable
#---------------------------------------------------------------------------
add_executable(openblt_stm32g473
    ${APP_SRCS}
    ${HAL_SRCS}
    ${STARTUP_SRC}
)

target_include_directories(openblt_stm32g473 PRIVATE
    ${LIB_INC_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(openblt_stm32g473
    openblt-core
    openblt-armcm4-stm32g4
    openblt-armcm4-stm32g4-gcc
)

#---------------------------------------------------------------------------
# Post-build: generate .srec, .map and print size summary
#---------------------------------------------------------------------------
# Also copy the ELF into the fixed build folder so everything is together.
add_custom_command(TARGET openblt_stm32g473 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:openblt_stm32g473> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/openblt_stm32g473.elf
    COMMAND ${CMAKE_OBJCOPY} -O srec $<TARGET_FILE:openblt_stm32g473> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/openblt_stm32g473.srec
    COMMAND ${CMAKE_OBJDUMP} -x $<TARGET_FILE:openblt_stm32g473> > ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/openblt_stm32g473.map
    COMMAND ${CMAKE_SIZE} -B -d $<TARGET_FILE:openblt_stm32g473>
    COMMENT "Post-build: copying ELF and generating srec/map"
)
