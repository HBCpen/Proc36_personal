cmake_minimum_required(VERSION 3.16)
project(proc36_personal CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wformat
)

add_library(proc36_lib
    src/lib/field.cpp
    src/lib/problem.cpp
)

target_include_directories(proc36_lib
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

add_executable(local_runner
    src/tools/local_runner.cpp
)

target_link_libraries(local_runner PRIVATE proc36_lib)
