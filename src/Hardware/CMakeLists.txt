# 指定CMake最低版本
cmake_minimum_required(VERSION 3.22)

# ------------------------------------------------------------
# 定义静态库目标
add_library(hardware_driver STATIC)

# 添加源文件
target_sources(hardware_driver PRIVATE
    src/g473_flash.c
    src/hardware.c
    src/as5047.c
    src/stm32_pwm.c
    src/rtc_time.c
)

target_include_directories(hardware_driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc 
)

target_link_libraries(hardware_driver PUBLIC 
   stm32cubemx 
)
