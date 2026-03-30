# 指定CMake最低版本
cmake_minimum_required(VERSION 3.22)

# ------------------------------------------------------------
# 定义静态库目标
add_library(cia402 STATIC)

# 添加源文件
target_sources(cia402 PRIVATE
    cia402.c
    cia402_state.c
)

target_include_directories(cia402
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/ 
)

target_link_libraries(cia402 PUBLIC 
   sm
   motor
)
