# 指定CMake最低版本
cmake_minimum_required(VERSION 3.22)

add_library(sm STATIC)

target_sources(sm PRIVATE
        statemachine.c
)

target_include_directories(sm PRIVATE

)

target_include_directories(sm PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sm PRIVATE
)
