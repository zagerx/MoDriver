# 指定CMake最低版本
cmake_minimum_required(VERSION 3.22)

# ------------------------------------------------------------
# 定义静态库目标
add_library(cmsis-dsp-v1.10.0 STATIC)

# 添加源文件
target_sources(cmsis-dsp-v1.10.0 PRIVATE
    Source/ControllerFunctions/arm_sin_cos_f32.c
    Source/CommonTables/arm_common_tables.c
)

target_include_directories(cmsis-dsp-v1.10.0
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Include 
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
)
target_include_directories(cmsis-dsp-v1.10.0
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/PrivateInclude 

       )

target_link_libraries(cmsis-dsp-v1.10.0 PRIVATE
#    stm32cubemx 
)
