# 指定CMake最低版本
cmake_minimum_required(VERSION 3.22)

add_library(motor STATIC)

target_sources(motor PRIVATE
        driver/inverter.c
        driver/feedback.c
        driver/currsmp.c
        lib/calibration/calibration.c
        lib/calibration/current_calibration.c
        lib/calibration/encoder_calibration.c
        lib/trajectory_plan.c
        lib/motor_state.c
        lib/svpwm.c
        lib/foc_data.c
        lib/foc.c
        motor_config.c
        motor.c
)

target_include_directories(motor PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/driver
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/calibration
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
        ${MOTORLIB_PARAMCONFIG_PATH}
)

target_include_directories(motor PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/interface
)

# motor 依赖 CMSIS-DSP
target_link_libraries(motor PRIVATE
        cmsis-dsp-v1.10.0
        sm
)
