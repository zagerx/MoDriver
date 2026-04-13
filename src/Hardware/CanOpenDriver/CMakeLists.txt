# 指定CMake最低版本
cmake_minimum_required(VERSION 3.22)

# ------------------------------------------------------------
# 定义静态库目标
add_library(canopennode_driver STATIC)

# 添加源文件
target_sources(canopennode_driver PRIVATE
    CO_driver_STM32.c
)

target_include_directories(canopennode_driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/CANopenNode

)
target_link_libraries(canopennode_driver PRIVATE 
   hardware_driver 
#    canopennode  # 链接CANopen协议栈以获取crc16_ccitt和CO_storage_init实现
)
