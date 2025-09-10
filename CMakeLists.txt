cmake_minimum_required(VERSION 3.10)
project(mecanum LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(mecanum STATIC 
    src/wheel_control.c
    src/encoder.c
)

target_include_directories(mecanum PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mecanum PRIVATE pigpio pthread rt)

target_compile_features(mecanum PRIVATE c_std_11)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
endif()
